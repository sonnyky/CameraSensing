#include <librealsense2/rs.hpp> 
#include <iostream>
#include <string>
#include <map>
#include <objbase.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#pragma once

using namespace rs2;
using namespace std;
using namespace cv;

class realsense_capture
{
	struct view_port
	{
		map<int, rs2::frame> color_frame;
		map<int, rs2::frame> depth_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		string device_number;
	};

	

public:
	realsense_capture();
	~realsense_capture();

	void set_init_flag(bool flag);
	bool get_init_flag();

	// Just to test that the plugin is working and a class object is instantiated
	const char * plugin_name = "tinker realsense plugin for Unity";
	const char * get_plugin_name();

	// Text messages for error and debugging purposes
	const char * error_message = "No error";
	const char * get_error_message();
	void set_error_message(std::string error);

	// Device control methods
	void open_device();
	LPSTR* list_devices();
	void close_all_devices();
	void remove_devices();
	void destroy_class();

	// Obtaining depth values
	void poll_frames();
	const uint16_t* get_sandbox_depth();

	// Obtaining color frames
	void realsense_capture::get_color_image(unsigned char * data, int &width, int &height);

	void setup_processing_params();
	void setup_default_params();

	// Calibration helper methods
	const float * calc_homography(float proj_tl_x, float proj_tl_y, float proj_tr_x, float proj_tr_y, float proj_bl_x, float proj_bl_y, float proj_br_x, float proj_br_y,
		float image_tl_x, float image_tl_y, float image_tr_x, float image_tr_y, float image_bl_x, float image_bl_y, float image_br_x, float image_br_y, int &size);

private:
	bool init_success_flag;

	// Realsense context
	context ctx;

	// Mutex lockto prevent race conditions
	mutex _mutex;

	// Variable to manage connected devices and their streams
	map<string, view_port> _devices;

	// Must be executed before any processing can happen
	void enable_device(device dev);

	/************************************************** OpenCV Variables */

	// Helper methods
	int realsense_capture::stream_count();
	size_t realsense_capture::device_count();
};