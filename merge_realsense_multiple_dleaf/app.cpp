﻿#include "app.h"
#include "util.h"
#include <thread>
#include <chrono>

#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp> 

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

using namespace std;
using namespace rs2;


// Constructor
Capture::Capture()
{
}

// Destructor
Capture::~Capture()
{
	// Finalize
	finalize();
}

void Capture::initialize() {

	if (dleaf_show_detection_window) {
		// OpenCV windows for displaying camera streams
		for (int i = 0; i < device_count(); i++) {
			string ss_color, ss_depth;
			//ss_color = "Color";
			ss_depth = "Depth";
			//string color_name = ss_color.append(to_string(i));
			string depth_name = ss_depth.append(to_string(i));
			//cvNamedWindow(color_name.c_str(), CV_WINDOW_AUTOSIZE);
			cvNamedWindow(depth_name.c_str(), CV_WINDOW_AUTOSIZE);
		}
	}

	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = true;
	params.filterByColor = true;
	params.blobColor = 255;

	maxRadius = sqrt((360 * 360) + (640 * 640));
	cout << "maxRadius when init : " << to_string(maxRadius) << endl;

	cout << "current minArea : " << to_string(params.minArea) << endl;
	cout << "current adjustment : " << to_string(adjustment_);
	can_send_data = false;

	d = SimpleBlobDetector::create(params);

}

void Capture::set_server_ip(const char * host) {
	host_name = host;
}

void Capture::set_detection_params(int lowDistMin, int lowDistMax, int maxDistMin, int maxDistMax, int minBlobArea, int maxBlobArea, int erosionSize, int adjustment) {
	params.minArea = minBlobArea;

	low_dist_min = lowDistMin;
	low_dist_max = lowDistMax;
	high_dist_min = maxDistMin;
	high_dist_max = maxDistMax;
	adjustment_ = adjustment;
	params.maxArea = maxBlobArea;
	erosion_size = erosionSize;
}

void Capture::init_morph_element() {
	element = getStructuringElement(cv::MORPH_CROSS,
		cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		cv::Point(erosion_size, erosion_size));
}


void Capture::set_default_params() {
	params.minArea = 500;
	params.maxArea = 10000;
	low_dist_min = 1800;
	low_dist_max = 1600;
	high_dist_min = 1200;
	high_dist_max = 1000;
	erosion_size = 6;
	adjustment_ = 20;
	dleaf_show_detection_window = false;
	cout << "this is the value when setting first time : " << dleaf_show_detection_window << "\n"<< endl;
	homography = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
}

void Capture::set_detection_display_param(bool value)
{
	dleaf_show_detection_window = value;
}

void Capture::set_homography_matrix(Mat mat) {
	homography = mat;
}

void Capture::read_homography_file() {
	// ASSUMPTION : file is in the same folder
	FileStorage fs("homography.xml", FileStorage::READ);
	fs["Homography"] >> homography;
}

void Capture::finalize() {

}

void Capture::run()
{
	update();
}

void Capture::enable_device(rs2::device dev)
{
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

	// If we make it here, there is at least a new camera plugged in
	std::ifstream calib_file("calib_file.json", std::ifstream::binary);
	Json::Value calib_settings;
	calib_file >> calib_settings;

	Point2f pos_val, detect_limit, overlap;

	// Set default values
	pos_val.x = 0; pos_val.y = 0; detect_limit.x = 1280; detect_limit.y = 720; overlap.x = 0; overlap.y = 0;

	for (int i = 0; i < calib_settings["devices"].size(); i++) {
		// Check if the serial number is listed
		if (calib_settings["devices"][i]["serial_number"].asString() == serial_number) {
			// Found matching entry
			pos_val.x = calib_settings["devices"][i]["pos_x"].asFloat();
			pos_val.y = calib_settings["devices"][i]["pos_y"].asFloat();
			detect_limit.x = calib_settings["devices"][i]["limit_x"].asFloat();
			detect_limit.y = calib_settings["devices"][i]["limit_y"].asFloat();
			overlap.x = calib_settings["devices"][i]["overlap_x"].asFloat();
			overlap.y = calib_settings["devices"][i]["overlap_y"].asFloat();
		}
	}

	// Create a pipeline from the given device
	rs2::pipeline p;
	rs2::config c;
	c.enable_device(serial_number);

	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);

	// Hold it internally
	_devices.emplace(serial_number, view_port{ {},{}, p, profile, serial_number, pos_val, overlap, detect_limit });
	cout << "Device Added : " << serial_number << endl;

	initialize();
}

void Capture::remove_devices(const rs2::event_information& info)
{
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over the list of devices and check if it was disconnected
	auto itr = _devices.begin();
	while (itr != _devices.end())
	{
		if (info.was_removed(itr->second.profile.get_device()))
		{
			itr = _devices.erase(itr);
		}
		else
		{
			++itr;
		}
	}
}

size_t Capture::device_count()
{
	//std::lock_guard<std::mutex> lock(_mutex); //this is not necessary unless another class is running simultaneously.
	return _devices.size();
}

int Capture::stream_count()
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

// Update Data
void Capture::update()
{
	poll_frames();
	auto total_number_of_streams = stream_count();

	if (total_number_of_streams == 0)
	{
		cout << "No streams available" << endl;
		return;
	}

	if (device_count() == 1)
	{
		//cout << "There is only one camera" << endl;
	}

	// Update Color
	//updateColor();
	updateDepth();
}

void Capture::poll_frames()
{
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

// Update Color
inline void Capture::updateColor()
{
	std::lock_guard<std::mutex> lock(_mutex);
	int device_no = 0;
	for (auto&& view : _devices)
	{
		string ss_color_name = "Color"; string ss_depth_name = "Depth";

		// For each device get its pipeline
		for (auto&& id_to_frame : view.second.color_frame)
		{
			// If the frame is available
			if (id_to_frame.second)
			{
				string color_name = ss_color_name.append(to_string(device_no));
				string depth_name = ss_depth_name.append(to_string(device_no));
				// Creating OpenCV Matrix from a color image
				Mat color(Size(640, 480), CV_8UC3, (void*)id_to_frame.second.get_data(), Mat::AUTO_STEP);
				cvtColor(color, color, COLOR_BGR2RGB);
				// Display in a GUI
				imshow(color_name.c_str(), color);
			}

		}
		device_no++;
	}
}

inline void Capture::updateDepth()
{
	std::lock_guard<std::mutex> lock(_mutex);
	int device_no = 0;

	// Clear list of blob center points. This list holds detected people position
	positions.clear();
	serialized_positions.clear();
	buf_length = 0;
	can_send_data = false;

	zaboom::people_position data_to_send = zaboom::people_position();
	int total_data = 0;
	for (auto&& view : _devices)
	{
		string ss_depth_name = "Depth";

		// For each device get its pipeline
		for (auto&& id_to_frame : view.second.depth_frame)
		{
			// If the frame is available
			if (id_to_frame.second)
			{
				string depth_name = ss_depth_name.append(to_string(device_no));
				pipeline_profile cur_pipeline_profile = view.second.profile;

				// Query frame size (width and height)
				const int w = id_to_frame.second.as<rs2::video_frame>().get_width();
				const int h = id_to_frame.second.as<rs2::video_frame>().get_height();

				cv::Mat depthImage = cv::Mat(h, w, CV_16U, (char*)id_to_frame.second.get_data());

				cv::Mat depthClone = depthImage.clone();
				// �Ώۋ����܂ł܂ł̃f�[�^��0-255�ɂ���
				inRange(depthImage, Scalar(high_dist_max), Scalar(high_dist_min), depthImage);
				depthImage.convertTo(depthImage, CV_8U);

				inRange(depthClone, Scalar(low_dist_max), Scalar(low_dist_min), depthClone);
				depthClone.convertTo(depthClone, CV_8U);

				bitwise_or(depthImage, depthClone, depthImage);
				// dilate image to fill out the gaps
				dilate(depthImage, depthImage, element);

				// Draw detected blobs as red circles.
				// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
				vector<KeyPoint> one_set_of_keypoints;
				d->detect(depthImage, one_set_of_keypoints);
				drawKeypoints(depthImage, one_set_of_keypoints, depthImage, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

				// TODO : apply transformation according to camera position
				for (int i = 0; i < one_set_of_keypoints.size(); i++) {

					int pos_x = one_set_of_keypoints[i].pt.x; int pos_y = one_set_of_keypoints[i].pt.y;

					if (pos_x > view.second.detection_limit.x || pos_y > view.second.detection_limit.y) {
						continue;
					}

					// positions are already adjusted to the main axes system
					// x and y for view.second.position are reversed because in the calib file, x denotes row and y denotes column
					// but in the image coordinates, x is the horizontal direction and y is vertical
					vector<Point2f> img, dst;

					//ASSUMPTION : image is 1280 x 720
					double pos_from_center_x = pos_x - 640; double pos_from_center_y = pos_y - 360;
					double angle_from_center = atan2(pos_from_center_y, pos_from_center_x);
					double magnitude = sqrt((pos_from_center_x*pos_from_center_x) + (pos_from_center_y * pos_from_center_y));
					//cout << "maxRadius : " << to_string(maxRadius)<< endl;
					double adjusted_magnitude = magnitude - ((magnitude / maxRadius) * (double)adjustment_);

					//cout << "adjusted_magnitude" << to_string(adjusted_magnitude)<< endl;

					int adjusted_x = (int)(adjusted_magnitude * cos(angle_from_center)) + 640;
					int adjusted_y = (int)(adjusted_magnitude * sin(angle_from_center)) + 360;
					/*	cout << "adjusted_x" << to_string(adjusted_x) <<  endl;
						cout << "adjusted_y" << to_string(adjusted_y) << endl;*/

					img.push_back(Point((float)(adjusted_x + (view.second.position.y * 1280) - view.second.overlap.x), (float)(adjusted_y + (view.second.position.x * 720) - view.second.overlap.y)));
					perspectiveTransform(img, dst, homography);

					data_to_send.add_x((int)dst[0].x);
					data_to_send.add_y((int)dst[0].y);
					total_data++;
				}

				// Display in a GUI
				if (dleaf_show_detection_window) {
					imshow(depth_name.c_str(), depthImage);
				}
			}
		}
		device_no++;
	}

	// Send through socket. data_to_send contains positions from multiple connected devices
	string newBuf;

	if (total_data == 0) {
		//cout << "ByteSize object : " << to_string(data_to_send.ByteSize()) << endl;
		data_to_send.add_x(0);
		data_to_send.add_y(0);
	}
	data_to_send.SerializeToString(&newBuf);
	send_length_to_socket(data_to_send.ByteSize());
	send_data(newBuf.data(), data_to_send.ByteSize());
	if (iResult <= 0) {
		close_socket();
	}

}

void Capture::mouseCallback(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN)
	{
		Capture* self = static_cast<Capture*>(userdata);
		self->doMouseCallback(event, x, y, flags);
	}
}

void Capture::setup_socket() {

	WSADATA wsaData;

	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	buf_length = 0;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		throw "Cannot initialize Winsock";
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	cout << "connecting to : " << host_name << endl;

	// Resolve the server address and port
	iResult = getaddrinfo(host_name, port, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		throw "Cannot getaddrinfo";
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			throw "socket failed ";
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		throw "Server not found";
	}

}

void Capture::send_data(const char * data, int size) {

	iResult = send(ConnectSocket, data, size, 0);
	if (iResult <= 0) return;

	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		throw "send failed";
	}
	printf("Bytes Sent: %ld\n", iResult);
}

void Capture::send_length_to_socket(int buf_to_send) {
	// Variable out of scope ?
	int buf_copy = buf_to_send;
	// Send the message to server
	iResult = send(ConnectSocket, (const char *)&buf_copy, sizeof(int), 0);

	if (iResult <= 0) return;

	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		throw "send failed";
	}
	//printf("Bytes Sent: %ld\n", iResult);

}

bool Capture::receive_length_confirmation() {
	if (iResult <= 0) return false;
	int data_echo;
	iResult = recv(ConnectSocket, (char *)&data_echo, buf_length, 0);
	if (iResult > 0) {
		//printf("Bytes received: %d and data echo : %d\n", iResult, (int)data_echo);
		if (data_echo == buf_length) return true;
	}
	else if (iResult == 0)
		printf("Connection closed\n");
	else
		printf("recv failed with error: %d\n", WSAGetLastError());

	return false;
}

void Capture::close_socket() {
	closesocket(ConnectSocket);
	WSACleanup();
}

void Capture::doMouseCallback(int event, int x, int y, int flags) {
	if (flags == (cv::EVENT_FLAG_LBUTTON))
	{
		std::cout << "Left mouse clicked at : " << x << ", " << y << std::endl;
		topLeftImage.x = x;
		topLeftImage.y = y;
	}

	if (flags == (cv::EVENT_FLAG_LBUTTON + cv::EVENT_FLAG_SHIFTKEY))
	{
		std::cout << "Shift+Left clicked at : " << x << ", " << y << std::endl;
		topRightImage.x = x;
		topRightImage.y = y;
	}

	if (flags == (cv::EVENT_FLAG_RBUTTON))
	{
		std::cout << "Right mouse clicked at : " << x << ", " << y << std::endl;
		bottomLeftImage.x = x;
		bottomLeftImage.y = y;
	}

	if (flags == (cv::EVENT_FLAG_RBUTTON + cv::EVENT_FLAG_SHIFTKEY))
	{

		std::cout << "Shift+Right clicked at : " << x << ", " << y << std::endl;
		bottomRightImage.x = x;
		bottomRightImage.y = y;
	}
}

void Capture::setRangePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY) {
	topLeft.x = topLeftX;
	topLeft.y = topLeftY;
	topRight.x = topRightX;
	topRight.y = topRightY;
	bottomLeft.x = bottomLeftX;
	bottomLeft.y = bottomLeftY;
	bottomRight.x = bottomRightX;
	bottomRight.y = bottomRightY;
	printf("Top left x : %f\n", topLeft.x);
	printf("Top left y : %f\n", topLeft.y);
	printf("Top right x : %f\n", topRight.x);
	printf("Top right y : %f\n", topRight.y);
	printf("Bottom left x : %f\n", bottomLeft.x);
	printf("Bottom left y : %f\n", bottomLeft.y);
	printf("Bottom right x : %f\n", bottomRight.x);
	printf("Bottom right y : %f\n", bottomRight.y);
}

void Capture::calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest) {
	printf("Calculating homography...\n Please redo clicking if the results are not as expected.\n Matrix will be output as text file.");
	Mat h = findHomography(pts_src, pts_dest);
	FileStorage file("homography.xml", 1, "UTF-8");
	file << "Homography" << h;
}