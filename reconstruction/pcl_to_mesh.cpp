#include "pcl_to_mesh.h"


Tinker::pcl_to_mesh::pcl_to_mesh():
	cloud1(new pcl::PointCloud<pcl::PointXYZ>), 
	cloud2(new pcl::PointCloud<pcl::PointXYZ>), 
	aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	GlobalTransform(Eigen::Matrix4f::Identity())
{
}

Tinker::pcl_to_mesh::~pcl_to_mesh()
{
}

void Tinker::pcl_to_mesh::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string fileName, bool useFile)
{
	if (useFile) {
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(fileName, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	}

	cout << "Size of cloud : " << cloud->size() << endl;
	if (cloud->size() == 0) return;

	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	pcl::copyPointCloud(*cloud_filtered, *cloud);

	
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(k_search_param);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures
	cout << "estimation finished" << endl;

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	cout << "concatenation finished" << endl;


	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	cout << "search tree finished" << endl;


	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	cout << "object initialization finished" << endl;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(search_radius);

	// Set typical values for the parameters
	gp3.setMu(mu);
	int val = max_nearest_neighbour + ((cloud->size()/cloud_size_limit) * max_nearest_neighbour);
	cout << "value of maximum nearest neighbors : " << val << endl;
	gp3.setMaximumNearestNeighbors(val);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	//pcl::io::saveOBJFile("mesh.obj", triangles);
	pcl::io::saveVTKFile("mesh_test.vtk", triangles);

}

void Tinker::pcl_to_mesh::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f & final_transform, bool downsample)
{
	//
 // Downsample for consistency and speed
 // \note enable this for large datasets
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	cout << "Size of cloud1 in pairAlign: " << cloud1->size() << endl;
	cout << "Size of cloud2 in pairAlign: " << cloud2->size() << endl;
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
		cout << "downsampled.." << endl;
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
		cout << "not downsampled.." << endl;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	cout << "Compute surface normals and curvature.." << endl;

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);
	cout << "setRescaleValues.." << endl;
	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(1e-10);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(max_correspondence_distance);
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	cout << "Align.." << endl;

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, sourceToTarget;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(100);
	reg.setRANSACOutlierRejectionThreshold(0.6);

	// Estimate
	reg.setInputSource(points_with_normals_src);
	reg.align(*reg_result);
	
  // Get the transformation from target to source
	Ti = reg.getFinalTransformation();
	sourceToTarget = Ti;

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_src, *output, sourceToTarget);

	//add the source to the transformed target
	*output += *cloud_tgt;

	final_transform = sourceToTarget;
	std::cout << "has converged:" << reg.hasConverged() << " score: " <<
		reg.getFitnessScore() << std::endl;
	cout << "final transformation : " << endl;
	cout << reg.getFinalTransformation() <<endl;
}

void Tinker::pcl_to_mesh::add_to_cloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cloud1->clear();

	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	pcl::copyPointCloud(*cloud_filtered, *cloud1);
	cout << "Size of cloud1: " << cloud1->size() << endl;
}

void Tinker::pcl_to_mesh::add_to_cloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cloud2->clear();

	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	pcl::copyPointCloud(*cloud_filtered, *cloud2);
	cout << "Size of cloud2: " << cloud2->size() << endl;
}

void Tinker::pcl_to_mesh::align_and_save_clouds()
{
	cout << "starting aligning process" << endl;
	aligned_cloud->clear();

	cout << "defining transform matrices" << endl;
	Eigen::Matrix4f pairTransform;

	cout << "Running align algorithm" << endl;
	pairAlign(cloud1, cloud2, aligned_cloud, pairTransform, true);

	//GlobalTransform *= pairTransform;

	pcl::io::savePCDFile("aligned.pcd", *aligned_cloud, true);
	generate_mesh_from_file();
}



void Tinker::pcl_to_mesh::align_clouds()
{
	PointCloud::Ptr temp(new PointCloud);
	cout << "defining transform matrices" << endl;
	Eigen::Matrix4f pairTransform;

	cout << "Running align algorithm" << endl;
	pairAlign(cloud1, cloud2, temp, pairTransform, true);

	if (pairTransform == Eigen::Matrix4f::Identity()) {
		return; //failed to converge
	}

	//pcl::transformPointCloud(*temp, *aligned_cloud, GlobalTransform);

	// set cloud 1 to be the aligned cloud
	cloud1->clear();
	pcl::copyPointCloud(*temp, *cloud1);
	pcl::copyPointCloud(*cloud1, *aligned_cloud);
	//string saveName = "aligned" + to_string(clock()) + ".pcd";
	//pcl::io::savePCDFile(saveName, *aligned_cloud, true);
	//GlobalTransform *= pairTransform;

}

void Tinker::pcl_to_mesh::clear_aligned_clouds()
{
	
	aligned_cloud->clear();
}

void Tinker::pcl_to_mesh::save_and_generate_mesh() {

	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(aligned_cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);


	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	vox.setInputCloud(cloud_filtered);
	vox.setLeafSize(0.01f, 0.01f, 0.01f);
	vox.filter(*output);

	pcl::io::savePCDFile("aligned.pcd", *output, true);
	generate_mesh_from_file();
}

void Tinker::pcl_to_mesh::generate_mesh_from_file()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	estimate(cloud, "aligned.pcd", true);
}

void Tinker::pcl_to_mesh::continuous_scan_store_aligned_as_cloud1()
{
	align_clouds();
}

void Tinker::pcl_to_mesh::setup_reconstruction_parameters()
{
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file("parameters.xml");
	cout << "Load result: " << result.description() << endl;

	pugi::xpath_query query_k_search("/params/reconstruction_params/k_search_param");
	k_search_param = stoi(query_k_search.evaluate_string(doc));
	cout << "k_search_param : " << k_search_param << endl;

	pugi::xpath_query query_search_radius("/params/reconstruction_params/search_radius");
	search_radius = stod(query_search_radius.evaluate_string(doc));
	cout << "search_radius : " << search_radius << endl;

	pugi::xpath_query query_mu("/params/reconstruction_params/mu");
	mu = stod(query_mu.evaluate_string(doc));
	cout << "mu : " << mu << endl;

	pugi::xpath_query query_max_nearest_neighbour("/params/reconstruction_params/max_nearest_neighbour");
	max_nearest_neighbour = stoi(query_max_nearest_neighbour.evaluate_string(doc));
	cout << "max_nearest_neighbour : " << max_nearest_neighbour << endl;

	pugi::xpath_query query_cloud_size_limit("/params/reconstruction_params/cloud_size_limit");
	cloud_size_limit = stoi(query_cloud_size_limit.evaluate_string(doc));
	cout << "cloud_size_limit : " << cloud_size_limit << endl;

	pugi::xpath_query query_max_correspondence_distance("/params/reconstruction_params/align_max_corr_distance");
	max_correspondence_distance = stod(query_max_correspondence_distance.evaluate_string(doc));
	cout << "max_correspondence_distance : " << max_correspondence_distance << endl;
}
