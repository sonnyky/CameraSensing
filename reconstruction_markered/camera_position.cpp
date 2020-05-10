#include "camera_position.h"

CameraPosition::CameraPosition():aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
	viewer.addCoordinateSystem(0.1);
	viewer.setBackgroundColor(0, 0, 0);
}

CameraPosition::~CameraPosition()
{
}

inline void Create4x4MatrixFromRt(Mat rot, Mat trans, Mat *concat) {
	// rot is a 3x3, trans is a 3x1
	Mat temp;
	Mat padding = Mat::zeros(1, 4, CV_64F);
	padding.at<double>(0, 3) = 1.0;

	hconcat(rot, trans, temp);
	vconcat( temp, padding, *concat);
}


void CameraPosition::DetectChessboard(Mat image)
{
	chessboard_image_points_.clear();
	bool foundChessCorners = findChessboardCorners(image, board_size_, chessboard_image_points_,
		CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

	if (foundChessCorners) {
		drawChessboardCorners(image, board_size_, Mat(chessboard_image_points_), foundChessCorners);
		imshow("Chessboard", image);
		solvePnP(chessboard_world_points_, chessboard_image_points_, camera_matrix_, dist_coeffs_, board_rot_, board_trans_);

		board_rot_matrix_ = Mat::eye(3, 3, CV_64F);
		Rodrigues(board_rot_, board_rot_matrix_);

		// Finding the fundamental matrix from image to camera
		Mat concatenated;
		Create4x4MatrixFromRt(board_rot_matrix_, board_trans_, &concatenated);
		Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eigen_concat(concatenated.ptr<double>(), concatenated.rows, concatenated.cols);
		current_camera_pose_from_image_ = eigen_concat.cast<float>();
	}

}


bool CameraPosition::Initialize(string calib_file_name, Size board_size)
{
	FileStorage fs(calib_file_name, FileStorage::READ);
	if (!fs.isOpened()) return false;

	fs["camera_matrix"] >> camera_matrix_;
	fs["distortion_coefficients"] >> dist_coeffs_;

	board_size_ = board_size;

	SetupBoardObjectPoints();
	return true;
}

void CameraPosition::SetupBoardObjectPoints()
{
	chessboard_world_points_.clear();
	for (int i = 0; i < board_size_.height; i++) {
		for (int j = 0; j < board_size_.width; j++) {
			chessboard_world_points_.push_back(cv::Point3f(float(j * square_size_), float(i * square_size_), 0));
		}
	}
}

void CameraPosition::SaveCurrentPoseAndCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	CameraPosePointCloud current;
	current.cloud = cloud;
	current.camera_pose = current_camera_pose_from_image_;
	point_clouds_wrt_camera_pose_.push_back(current);
	cout << "the size of input cloud : " << cloud->size() << endl;
	cout << "the size of saved cloud : " << current.cloud->size() << endl;
}

void CameraPosition::ClearAlignedCloud() {
	aligned_cloud_->clear();
}

void CameraPosition::AlignAndReconstructClouds()
{
	if (point_clouds_wrt_camera_pose_.size() < 2) {
		cout << "point clouds list not enough for reconstruction" <<endl;
		return;
	}
	cout << "check the pose and cloud " << endl;
	cout << "cloud size : " << point_clouds_wrt_camera_pose_[0].cloud->size() << endl;
	cout << "pose : " << endl;

	cout << point_clouds_wrt_camera_pose_[0].camera_pose << endl;
	Eigen::Matrix4f inverse = point_clouds_wrt_camera_pose_[0].camera_pose.inverse();

	*aligned_cloud_ += *(point_clouds_wrt_camera_pose_[0].cloud);

	cout << "inverse :" << endl;
	cout << inverse << endl;

	cout << " test shouldBeIdentity :" << endl;
	Eigen::Matrix4f shouldBeIdentity = point_clouds_wrt_camera_pose_[0].camera_pose * inverse;
	cout << shouldBeIdentity << endl;

	// T1 = M * T2
	// To get the transformation matrix M between two camera poses, we need to do M = T1 * inv(T2)
	for (int i = 1; i < point_clouds_wrt_camera_pose_.size(); i++) {
		
		Eigen::Matrix4f transformationMatrixFromBase = point_clouds_wrt_camera_pose_[i].camera_pose * inverse;
		cout << "transformationMatrixFromBase : " << endl;
		cout << transformationMatrixFromBase << endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*point_clouds_wrt_camera_pose_[i].cloud, *transformed_cloud, transformationMatrixFromBase.inverse());
		point_clouds_wrt_camera_pose_[i].cloud = transformed_cloud;

		*aligned_cloud_ += *transformed_cloud;
		
	}

	SaveAlignedCloud();

}

void CameraPosition::SaveSingleShotCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	RemoveStatisticalOutliers(cloud, cloud_filtered);
	visualize(cloud_filtered);
	pcl::io::savePCDFile("single_shot.pcd", *cloud_filtered, true);
}

void CameraPosition::SaveAlignedCloud()
{
	pcl::io::savePCDFile("aligned_markered.pcd", *aligned_cloud_, true);
}

void CameraPosition::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_rgb->resize(cloud->size());

	for (int i = 0; i < cloud->size(); i++) {
		cloud_rgb->points[i].x = cloud->points[i].x;
		cloud_rgb->points[i].y = cloud->points[i].y;
		cloud_rgb->points[i].z = cloud->points[i].z;

	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud_rgb, 0, 0, 255); //blue
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "cloud_rgb");
	viewer.spin();

}

void CameraPosition::ClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// the output cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;
	}

}

void CameraPosition::RemoveStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

}
