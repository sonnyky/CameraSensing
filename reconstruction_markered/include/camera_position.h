#ifndef __CAM_POS__
#define __UTIL__
#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>


#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>

#include "pugixml.hpp";

using namespace std;
using namespace cv;
using namespace pcl;

class CameraPosition {

	struct CameraPosePointCloud {
		Eigen::Matrix4f camera_pose;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	};

public:
	CameraPosition();
	~CameraPosition();

	void DetectChessboard(Mat image);

	// Chessboard values
	Size board_size_;
	float square_size_ = 0.0340; // in meters
	float aspect_ratio_ = 1.0;
	Mat camera_matrix_, dist_coeffs_;
	string calib_file_name_;

	// Board rotation and translation as detected by opencv
	cv::Mat board_rot_;
	Mat board_rot_matrix_;
	cv::Mat board_trans_;
	Eigen::Matrix4f current_camera_pose_from_image_;
	
	bool Initialize(string calib_file_name, Size board_size);
	void SetupBoardObjectPoints();
	Eigen::Matrix4f GetCurrentCameraPoseMatrix(){ return current_camera_pose_from_image_; };

	// [0] is the base pose
	void SaveCurrentPoseAndCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	void ClearAlignedCloud();

	void AlignAndReconstructClouds();

	void SaveSingleShotCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	void SaveAlignedCloud();

private:
	vector<Point2f> chessboard_image_points_;
	vector<Point3f> chessboard_world_points_;
	vector<CameraPosePointCloud> point_clouds_wrt_camera_pose_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;


};

#endif // __CAM_POS__