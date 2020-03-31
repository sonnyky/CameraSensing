#include "pcl_to_mesh.h"


Tinker::pcl_to_mesh::pcl_to_mesh()
{
}

Tinker::pcl_to_mesh::~pcl_to_mesh()
{
}

// Estimate normals
void Tinker::pcl_to_mesh::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/*pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("bun.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);*/
	//* the data should be available in cloud
	cout << "Size of cloud : " << cloud->size() << endl;

	cout << "loading finished" << endl;
	if (cloud->size() == 0) return;
	

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
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
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
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

	pcl::io::saveVTKFile("mesh.vtk", triangles);

}
