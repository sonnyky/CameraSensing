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

class recognition
{

public:
	recognition();
	~recognition();

	// Just to test that the plugin is working and a class object is instantiated
	const char * plugin_name = "tinker webcam face recognition plugin for Unity";
	const char * get_plugin_name();

	// Text messages for error and debugging purposes
	const char * error_message = "No error";
	const char * get_error_message();
	void set_error_message(std::string error);
	void setup_camera();
	void get_color_image(unsigned char * data, int &width, int &height);
	void release_camera();
	void destroy_class();
	void detect_faces(unsigned char * input, unsigned char * processed, int width, int height);
	
private:
	bool init_success_flag;
	VideoCapture cap;
	CascadeClassifier face_cascade;
	const char * error = "no error";
};

