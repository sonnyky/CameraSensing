#include "camera_position.h"

CameraPosition::CameraPosition():aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
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
	pcl::io::savePCDFile("single_shot.pcd", *cloud, true);
}

void CameraPosition::SaveAlignedCloud()
{
	pcl::io::savePCDFile("aligned_markered.pcd", *aligned_cloud_, true);
}
