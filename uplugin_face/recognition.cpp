#include "recognition.h"
#include "detection.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

recognition::recognition()
{
	face_cascade.load("lbpcascade_frontalface.xml");
	if (face_cascade.empty()) {
		error = "cascade load error";
	}
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

void recognition::detect_faces(unsigned char * input, unsigned char * processed, int width, int height)
{
	Mat image(height, width, CV_8UC4);
	memcpy(image.data, input, height * width * 4);

	Mat frame_gray;
	cvtColor(image, frame_gray, COLOR_RGBA2GRAY);
	equalizeHist(frame_gray, frame_gray);

	std::vector<Rect> faces;
	face_cascade.detectMultiScale(frame_gray, faces);
	for (size_t i = 0; i < faces.size(); i++)
	{
		rectangle(image, faces[i], Scalar(255.0, 0.0, 255.0, 1.0), 2, 8, 0);
	}

	memcpy(processed, image.data, height * width * 4);

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