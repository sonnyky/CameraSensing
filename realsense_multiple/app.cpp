#include "app.h"
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

	// OpenCV windows for displaying camera streams
	for (int i = 0; i < device_count(); i++) {
		string ss_color, ss_depth;
		ss_color = "Color";
		ss_depth = "Depth";
		string color_name = ss_color.append(to_string(i));
		string depth_name = ss_depth.append(to_string(i));
		cvNamedWindow(color_name.c_str(), CV_WINDOW_AUTOSIZE);
		cvNamedWindow(depth_name.c_str(), CV_WINDOW_AUTOSIZE);
	}

	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = true;
	params.maxArea = 10000;
	params.filterByColor = true;
	params.blobColor = 255;

	cout << "current minArea : " << to_string(params.minArea) << endl;
	cout << "current threshold : " << to_string(distanceThreshold);

	d = SimpleBlobDetector::create(params);
	//TODO : set mouse callback on all windows ? maybe just depth ? to calibrate detection area.
	//cv::setMouseCallback("Color", mouseCallback, this);
}

void Capture::set_detection_params(int distThres, int minBlobArea) {
	params.minArea = minBlobArea;
	distanceThreshold = distThres;
}

void Capture::set_default_params() {
	params.minArea = 500;
	distanceThreshold = 1500;
}

void Capture::finalize() {

}

void Capture::run()
{
	while (true) {
		update();
		if (waitKey(1) == 99) {
			vector<Point2f> pts_src, pts_dest;
			pts_src.push_back(topLeftImage); pts_src.push_back(topRightImage); pts_src.push_back(bottomRightImage); pts_src.push_back(bottomLeftImage);
			pts_dest.push_back(topLeft); pts_dest.push_back(topRight); pts_dest.push_back(bottomRight); pts_dest.push_back(bottomLeft);
			calcHomographyMatrix(pts_src, pts_dest);
		}else if(waitKey(1) == 113) break;

	}
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
	// Create a pipeline from the given device
	rs2::pipeline p;
	rs2::config c;
	c.enable_device(serial_number);

	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);

	// Hold it internally
	_devices.emplace(serial_number, view_port{ {},{}, p, profile, serial_number });
	cout << "Device Added : "<< serial_number << endl;

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
	updateColor();
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

	//TODO : Clear list of blob center points. This list holds detected people position

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

				// ‘ÎÛ‹——£‚Ü‚Å‚Ü‚Å‚Ìƒf[ƒ^‚ð0-255‚É‚·‚é
				depthImage.setTo(0, depthImage > distanceThreshold);
				depthImage.convertTo(depthImage, CV_8U, 255.0 / distanceThreshold);

				// Draw detected blobs as red circles.
				// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
				keypoints.clear();
				d->detect(depthImage, keypoints);
				//cout <<"detected : " << to_string( keypoints.size()) << endl;
				drawKeypoints(depthImage, keypoints, depthImage, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


				// TODO : Append to a global list of detected blobs. need mutex ?
				// TODO : get the current pipeline device info
				// TODO : apply transformation according to camera position
				cur_pipeline_profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);


				// Display in a GUI
				imshow(depth_name.c_str(), depthImage);
			}
		}
		device_no++;
	}

}

void Capture::mouseCallback(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN)
	{
		Capture* self = static_cast<Capture*>(userdata);
		self->doMouseCallback(event, x, y, flags);
	}
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
	file <<"Homography" << h;
}