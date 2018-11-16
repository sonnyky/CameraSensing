#include "cv_depth.h"
#include "detection.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

cv_depth::cv_depth()
{
}

cv_depth::~cv_depth()
{

}

void cv_depth::set_error_message(std::string error) {
	error_message = error.c_str();
}

const char * cv_depth::get_plugin_name() {
	return plugin_name;
}

const char * cv_depth::get_error_message() {
	return error_message;
}

void cv_depth::destroy_class() {
	delete this;
}

/*
Calibration methods
*/
const float * cv_depth::calc_homography(UPoint * src, UPoint * dst, int length) {
	return _calc_homography(src, dst, length);
}