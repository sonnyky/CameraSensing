#include "realsense_capture.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once
using namespace rs2;

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

realsense_capture::realsense_capture()
{
	setup_default_params();
}

realsense_capture::~realsense_capture()
{

}

void realsense_capture::open_device() {
	
}

LPSTR* realsense_capture::list_devices() {
	int pSize = 4;

	// Instantiate new pointer to hold data everytime this method is called
	LPSTR* pData = (LPSTR*)CoTaskMemAlloc((pSize) * sizeof(LPSTR));

	for (int i = 0; i < pSize; i++) {
		pData[i] = (LPSTR)CoTaskMemAlloc(128);
		strcpy_s(pData[i], 128, "None");
	}

	int count = 0;
	// TODO : make sure the app behaves correctly when there are more than 4 devices
	for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
	{
		enable_device(dev);
		std::string name = "Unknown";
		if (dev.supports(RS2_CAMERA_INFO_NAME))
			name = dev.get_info(RS2_CAMERA_INFO_NAME);
		strcpy_s(pData[count], 128, name.c_str());
		count++;
	}

	return pData;
}

void realsense_capture::enable_device(device dev) {
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	std::lock_guard<std::mutex> lock(_mutex);

	if (_devices.find(serial_number) != _devices.end())
	{
		return; //already in
	}

	// Ignoring platform cameras (webcams, etc..)
	if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
	{
		return;
	}
	// Create a pipeline from the given device
	rs2::pipeline p;
	rs2::config c;
	c.enable_device(serial_number);

	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);

	// Hold it internally
	_devices.emplace(serial_number, view_port{ {}, {}, p, profile, serial_number });
}

void realsense_capture::remove_devices()
{
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over the list of devices and check if it was disconnected
	auto itr = _devices.begin();
	while (itr != _devices.end())
	{
		itr = _devices.erase(itr);
		
	}
}

void realsense_capture::setup_default_params() {
	
}

// TODO : setup camera post processing params
void realsense_capture::setup_processing_params() {
	
}

void realsense_capture::poll_frames() {
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over all device
	for (auto&& view : _devices)
	{
		// Ask each pipeline if there are new frames available
		rs2::frameset frameset;
		if (view.second.pipe.poll_for_frames(&frameset))
		{
			rs2::frame new_color_frame = frameset.get_color_frame();
			rs2::frame new_depth_frame = frameset.get_depth_frame();
			int stream_id = new_depth_frame.get_profile().unique_id();
			view.second.color_frame[stream_id] = new_color_frame; //update color view port with the new stream
			view.second.depth_frame[stream_id] = new_depth_frame; //update depth view port with the new stream
		}
	}
}

const uint16_t* realsense_capture::get_sandbox_depth() {
	poll_frames();
	rs2_error* e = 0;
	// depth frame is 1280 x 720. TODO : handle if depth frame size is different.
	// TODO get depth data from frame.
	const uint16_t * pointer_to_data;
	for (auto&& view : _devices)
	{
		// For each device get its pipeline
		for (auto&& id_to_frame : view.second.depth_frame)
		{
			// If the frame is available
			if (id_to_frame.second)
			{
				pipeline_profile cur_pipeline_profile = view.second.profile;

				// Query frame size (width and height)
				const int w = id_to_frame.second.as<rs2::video_frame>().get_width();
				const int h = id_to_frame.second.as<rs2::video_frame>().get_height();
				cv::Mat depthImage = cv::Mat(h, w, CV_16U, (char*)id_to_frame.second.get_data());

				pointer_to_data = (const uint16_t*)id_to_frame.second.get_data();				
				int depth = *pointer_to_data;
			}
		}
	}

	return pointer_to_data;
}

void realsense_capture::get_color_image(unsigned char * data, int &width, int &height) {
	poll_frames();
	// frame is assumed to be 640 x 480. TODO : handle if depth frame size is different.

	int device_no = 0;
	cv::Mat colorImage;
	if (_devices.size() != 1) {
		ofstream myfile;
		myfile.open("multiple_device_error.txt");
		myfile << "Not supporting more than one device for now\n";
		myfile.close();

		return;
	}

	auto total_number_of_streams = stream_count();

	if (total_number_of_streams == 0)
	{
		ofstream myfile;
		myfile.open("streams.txt");
		myfile << "No streams available\n";
		myfile.close();

		colorImage = cv::Mat(480, 640, CV_16U, Scalar(0));
		Mat rgbImg;
		cvtColor(colorImage, rgbImg, CV_GRAY2RGBA);
		memcpy(data, rgbImg.data, rgbImg.total() * rgbImg.elemSize());
		return;
	}

	for (auto&& view : _devices)
	{
		// For each device get its pipeline
		for (auto&& id_to_frame : view.second.color_frame)
		{
			// If the frame is available
			if (id_to_frame.second)
			{
				pipeline_profile cur_pipeline_profile = view.second.profile;
				colorImage = cv::Mat(Size(640, 480), CV_8UC3, (void*)id_to_frame.second.get_data(), Mat::AUTO_STEP);
			}
		}
		device_no++;
	}
	Mat rgbImg;
	cvtColor(colorImage, rgbImg, CV_BGR2RGBA);
	memcpy(data, rgbImg.data, rgbImg.total() * rgbImg.elemSize());
}



void realsense_capture::set_init_flag(bool flag) {
	init_success_flag = flag;
}

bool realsense_capture::get_init_flag() {
	return init_success_flag;
}


const char * realsense_capture::get_error_message() {
	return error_message;
}

void realsense_capture::set_error_message(std::string error) {
	error_message = error.c_str();
}

const char * realsense_capture::get_plugin_name() {
	return plugin_name;
}


void realsense_capture::close_all_devices() {
	// Go over all device
	for (auto&& view : _devices)
	{
		view.second.pipe.stop();
	}
}

void realsense_capture::destroy_class() {
	delete this;
}

/*
Calibration methods
*/
const float * realsense_capture::calc_homography(float proj_tl_x, float proj_tl_y, float proj_tr_x, float proj_tr_y, float proj_bl_x, float proj_bl_y, float proj_br_x, float proj_br_y,
	float image_tl_x, float image_tl_y, float image_tr_x, float image_tr_y, float image_bl_x, float image_bl_y, float image_br_x, float image_br_y, int &size) {

	vector<Point2f> proj_points;
	vector<Point2f> img_points;
	vector<float> homographyValues;
	Point2f point;
	point.x = (float)proj_tl_x; point.y = (float)proj_tl_y;
	proj_points.push_back(point);
	point.x = (float)proj_tr_x; point.y = (float)proj_tr_y;
	proj_points.push_back(point);
	point.x = (float)proj_bl_x; point.y = (float)proj_bl_y;
	proj_points.push_back(point);
	point.x = (float)proj_br_x; point.y = (float)proj_br_y;
	proj_points.push_back(point);

	point.x = (float)image_tl_x; point.y = (float)image_tl_y;
	img_points.push_back(point);
	point.x = (float)image_tr_x; point.y = (float)image_tr_y;
	img_points.push_back(point);
	point.x = (float)image_bl_x; point.y = (float)image_bl_y;
	img_points.push_back(point);
	point.x = (float)image_br_x; point.y = (float)image_br_y;
	img_points.push_back(point);

	Mat homography = findHomography(proj_points, img_points);
	for (int i = 0; i < homography.rows; i++) {
		for (int j = 0; j < homography.cols; j++) {
			homographyValues.push_back((float) homography.at<double>(i, j));
		}
	}
	
	size = homographyValues.size();
	return &homographyValues[0];
}


/*
Helper methods not exposed.
*/
size_t realsense_capture::device_count()
{
	//std::lock_guard<std::mutex> lock(_mutex); //this is not necessary unless another class is running simultaneously.
	return _devices.size();
}

int realsense_capture::stream_count()
{
	//std::lock_guard<std::mutex> lock(_mutex);
	int count = 0;
	for (auto&& sn_to_dev : _devices)
	{
		for (auto&& stream : sn_to_dev.second.depth_frame)
		{
			if (stream.second)
			{
				count++;
			}
		}
		for (auto&& stream : sn_to_dev.second.color_frame)
		{
			if (stream.second)
			{
				count++;
			}
		}
	}
	return count;
}