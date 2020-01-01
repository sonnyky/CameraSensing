#include "cv_util.h"
#include "homography.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

cv_util::cv_util()
{
}

cv_util::~cv_util()
{

}

void cv_util::set_error_message(std::string error) {
	error_message = error.c_str();
}

const char * cv_util::get_plugin_name() {
	return plugin_name;
}

const char * cv_util::get_error_message() {
	return error_message;
}

void cv_util::destroy_class() {
	delete this;
}

/*
Calibration methods
*/
const void cv_util::calc_homography(UPoint * src, UPoint * dst, int length, float * data) {
	return _calc_homography(src, dst, length, data);
}