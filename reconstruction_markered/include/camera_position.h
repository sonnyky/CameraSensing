#ifndef __CAM_POS__
#define __UTIL__
#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/mls.h>

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

	// Visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	void visualize(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::ConstPtr normals);

	// Cluster extraction
	void ClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// Data processing, smoothing, filtering etc.
	void RemoveStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	void AddNormalsToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

private:
	vector<Point2f> chessboard_image_points_;
	vector<Point3f> chessboard_world_points_;
	vector<CameraPosePointCloud> point_clouds_wrt_camera_pose_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;


};

#endif // __CAM_POS__