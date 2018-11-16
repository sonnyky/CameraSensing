#include <iostream>
#include <string>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#pragma once

struct UPoint {
	float x, y, z;
};

using namespace std;
using namespace cv;

class cv_depth
{

public:
	cv_depth();
	~cv_depth();

	// Just to test that the plugin is working and a class object is instantiated
	const char * plugin_name = "tinker cv_depth plugin for Unity";
	const char * get_plugin_name();

	// Text messages for error and debugging purposes
	const char * error_message = "No error";
	const char * get_error_message();
	void set_error_message(std::string error);
	const float * calc_homography(UPoint * src, UPoint * dst, int length);
	void destroy_class();
	
private:
	bool init_success_flag;

};

