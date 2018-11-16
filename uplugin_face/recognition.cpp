#include "recognition.h"
#include "detection.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

recognition::recognition()
{
}

recognition::~recognition()
{

}

void recognition::set_error_message(std::string error) {
	error_message = error.c_str();
}

const char * recognition::get_plugin_name() {
	return plugin_name;
}

const char * recognition::get_error_message() {
	return error_message;
}

void recognition::destroy_class() {
	delete this;
}

void recognition::setup_camera() {
	if (cap.isOpened()) {
		return;
	}
	else {
		cap.open(0);
	}
}

void recognition::get_color_image(unsigned char * data, int &width, int &height){
	Mat frame;
	cap >> frame;

	Mat rgbImg;
	cvtColor(frame, rgbImg, CV_BGR2RGBA);
	width = frame.cols;
	height = frame.rows;
	memcpy(data, rgbImg.data, rgbImg.total() * rgbImg.elemSize());
}

void recognition::release_camera() {
	cap.release();
}