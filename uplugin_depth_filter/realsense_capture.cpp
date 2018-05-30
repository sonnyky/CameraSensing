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

	if (count > 0) {
		// Initiate OpenCV image processing if at least one camera is found
		init_blob_detector();
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
	_devices.emplace(serial_number, view_port{ {}, p, profile, serial_number });
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
	params.minArea = 500;
	distanceThreshold = 1500;
}

void realsense_capture::setup_detection_params(int dist, int minBlobArea) {
	params.minArea = minBlobArea;
	distanceThreshold = dist;
}

void realsense_capture::init_blob_detector() {
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = true;
	params.maxArea = 10000;
	params.filterByColor = true;
	params.blobColor = 255;
	d = SimpleBlobDetector::create(params);
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
			//rs2::frame new_color_frame = frameset.get_color_frame();
			rs2::frame new_depth_frame = frameset.get_depth_frame();
			int stream_id = new_depth_frame.get_profile().unique_id();
			//view.second.color_frame[stream_id] = new_color_frame; //update color view port with the new stream
			view.second.depth_frame[stream_id] = new_depth_frame; //update depth view port with the new stream
		}
	}
}

const people_position* realsense_capture::get_depth(int &size) {
	poll_frames();
	// depth frame is 1280 x 720. TODO : handle if depth frame size is different.
	// TODO get depth data from frame.

	//std::lock_guard<std::mutex> lock(_mutex);
	int device_no = 0;
	int total_pos_size = 0;

	vector<people_position> _people_positions;

	// Clear all previous values of detected people.
	_people_positions.clear();
	//positions.clear();

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

				// TODO : Opencv blob detection
				cv::Mat depthImage = cv::Mat(h, w, CV_16U, (char*)id_to_frame.second.get_data());
				// Active sensing range is normalized to 0 - 255 range.
				depthImage.setTo(0, depthImage > distanceThreshold);
				depthImage.convertTo(depthImage, CV_8U, 255.0 / distanceThreshold);

				keypoints.clear();
				d->detect(depthImage, keypoints);
				int this_size = (int) keypoints.size();
				people_position this_pos;
				for (int i = 0; i < this_size; i++) {
					this_pos.x = keypoints[i].pt.x;
					this_pos.y = keypoints[i].pt.y;
					//positions.push_back(this_pos);
					_people_positions.push_back(this_pos);
					total_pos_size++;
				}

				// TODO : Append to a global list of detected blobs. need mutex ?
				// TODO : get the current pipeline device info
				// TODO : apply transformation according to camera position
				cur_pipeline_profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

			}
		}
		device_no++;
	}

	// Notify caller of the total size
	size = total_pos_size;

	// Observe that we are returning the same address everytime because this is the same member held by the class.

	return &_people_positions[0];
}

void realsense_capture::get_thresholded_image(unsigned char * data,  int &width, int &height) {
	poll_frames();
	// depth frame is assumed to be 1280 x 720. TODO : handle if depth frame size is different.

	int device_no = 0;
	cv::Mat depthImage;
	if (_devices.size() != 1) {
		cout << "Not supporting more than one device for now" << endl;
		return;
	}
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
				width = w;
				height = h;

				depthImage = cv::Mat(h, w, CV_16U, (char*)id_to_frame.second.get_data());
				// Active sensing range is normalized to 0 - 255 range.
				depthImage.setTo(0, depthImage > distanceThreshold);
				depthImage.convertTo(depthImage, CV_8U, 255.0 / distanceThreshold);
			}
		}
		device_no++;
	}
	Mat rgbImg;
	cvtColor(depthImage, rgbImg, CV_GRAY2RGBA);
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
	}
	return count;
}