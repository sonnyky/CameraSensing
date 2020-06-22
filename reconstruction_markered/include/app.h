#ifndef __APP__
#define __APP__

#pragma once

#include <Windows.h>
#include <comutil.h>
#include <iostream>
#include <cstdio>
#include <ctime>

#include <wtypes.h>
#include <comdef.h> 
#include <string>
#include <string.h>
#include <tchar.h>
#include <stdio.h>
#include "atlbase.h"
#include "atlwin.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <librealsense2/rs.hpp> 
#include<mutex>
#include <wrl/client.h>

#include "camera_position.h"

using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Capture {
	struct single_frame
	{
		rs2::frame color_frame;
		rs2::frame depth_frame;
	};

	struct single_frame_images
	{
		Mat color_image;
		Mat depth_image;
	};

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();
	void setup_capture_parameters();
	void ReadCloudFiles(string path_to_files);
	void SetCloudFromFiles();
	void EstimatePlanesFromFiles();

private :
	void initialize();
	void finalize();

	// Camera position tracker
	CameraPosition camera_position_;

	// Sensor initializations
	inline void initializeSensor();

	void poll_frames();

	rs2::pipeline m_pipeline;
	rs2::config cfg;
	rs2::frameset frames;
	std::mutex _mutex;

	vector<single_frame> views;

	bool stream_exists();

	rs2::frame current_color_frame;
	rs2::depth_frame current_depth_frame;

	Mat current_color_image;

	void TrackCameraPosition();

	// Drawing functions
	void update();
	void updateColor();
	void updateDepth();
	rs2::align align_to_color;

	bool save_pose_and_cloud = false;
	bool align_and_reconstruct = false;

	pcl_ptr points_to_pcl(const rs2::points& points);
	void SavePoseCloud();
	void SaveSingleCloud();
	pcl_ptr GeneratePointCloud();
	void AlignAndReconstruct();
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	int rs_filter_magnitude = 3;
	int time_diff = 2;

	vector<string> path_to_cloud_files_;

	// in meters
	float dist_limit_min = 0.0;
	float dist_limit_max = 4.0;
	float x_limit_min = -1.0;
	float x_limit_max = 1.0;
	float y_limit_min = 0.0;
	float y_limit_max = 1.0;
	string filter_field_name = "z";
};

#endif // __APP__