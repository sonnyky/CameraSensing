#ifndef __APP__
#define __APP__

#pragma once

#include <Windows.h>
#include <comutil.h>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include<mutex>
#include <wrl/client.h>
using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;


class Capture {
	
	struct view_port
	{
		std::map<int, rs2::frame> color_frame;
		std::map<int, rs2::frame> depth_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		string device_number;
	};

	struct single_frame 
	{
		Mat color_image;
		rs2::frame depth_frame;
	};


private :
	rs2::config cfg;
	rs2::frameset frames;
	rs2::colorizer color_map;

	// Active distance threshold
	int low_dist_min, low_dist_max, high_dist_min, high_dist_max;

	std::mutex _mutex;
	std::map<std::string, view_port> _devices;

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

	void Capture::enable_device(rs2::device dev);
	void Capture::remove_devices(const rs2::event_information& info);

	size_t Capture::device_count();
	vector<Mat> get_color_images();
	vector<Mat> get_depth_data();

	vector<single_frame> get_depth_and_color_frameset();

	// gets distance of a pixel in all connected cameras
	vector<float> get_all_distances_at_pixel(int x, int y);

	float get_distance_at_pixel(int x, int y, depth_frame depth_data_frame);

	void set_alignment(int a);

private :
	void initialize();
	void finalize();

	// Drawing functions
	void poll_frames();
	int Capture::stream_count();
	void update();
	inline void updateColor();
	inline void updateDepth();

	int alignment = 0;
	rs2::align align_to_color;

};

#endif // __APP__