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

void recognition::setup_camera(int id) {
	if (cap.isOpened()) {
		cap.release();
		return;
	}
	else {
		cap.open(id);
	}
}

void detect_shapes_in_image(Mat src, Mat res) {
	Mat gray;
	cvtColor(src, gray, COLOR_BGR2GRAY);

	Mat bw;
	Canny(gray, bw, 800, 850, 5, true);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(bw.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size() < 1) return;
	for (int i = 0; i < contours.size(); i++) {
		double contourLength = arcLength(contours[i], true);
		double area = contourArea(contours[i]);
		if (contourLength < 300 || area < 3000) continue;

		int result = _detect_shape(res, contours[i]);
	}

}

void recognition::get_color_image(unsigned char * data, int &width, int &height){
	Mat frame;
	cap >> frame;

	Mat clone = frame.clone();

	detect_shapes_in_image(frame, clone);
	flip(clone, clone, 1);
	Mat rgbImg;
	cvtColor(clone, rgbImg, CV_BGR2RGBA);
	width = clone.cols;
	height = clone.rows;
	memcpy(data, rgbImg.data, rgbImg.total() * rgbImg.elemSize());
}

void recognition::release_camera() {
	if (cap.isOpened()) {
		cap.release();
	}
}