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

	detector.test();

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

	cout << "current low_dist_min : " << to_string(low_dist_min) << endl;
	cout << "current low_dist_max : " << to_string(low_dist_max) << endl;
	cout << "current high_dist_min : " << to_string(high_dist_min) << endl;
	cout << "current high_dist_max : " << to_string(high_dist_max) << endl;

	element = getStructuringElement(cv::MORPH_CROSS,
		cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		cv::Point(erosion_size, erosion_size));
	
}

void Capture::set_detection_params(int lowDistMin, int lowDistMax, int highDistMin, int highDistMax, int erosionSize) {

	low_dist_min = lowDistMin;
	low_dist_max = lowDistMax;
	high_dist_min = highDistMin;
	high_dist_max = highDistMax;
	
	erosion_size = erosionSize;
}

void Capture::set_default_params() {

	low_dist_min = 1800;
	low_dist_max = 1600;
	high_dist_min = 1200;
	high_dist_max = 1000;
	erosion_size = 1;
}

void Capture::finalize() {

}

void Capture::run()
{
	while (true) {
		update();
		if(waitKey(1) == 113) break;

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
				cv::Mat depthClone = depthImage.clone();

				inRange(depthImage, Scalar(high_dist_max), Scalar(high_dist_min), depthImage);
				depthImage.convertTo(depthImage, CV_8U);

				inRange(depthClone, Scalar(low_dist_max), Scalar(low_dist_min), depthClone);
				depthClone.convertTo(depthClone, CV_8U);

				bitwise_or(depthImage, depthClone, depthImage);

				//Refine input image with dilation and erosion
				//erode(depthImage, depthImage, element);
				
				vector<shape> shapes_found;
				shapes_found = detector.detect_shape(depthImage);

				int num_of_shapes = shapes_found.size();
				
				cout << " Shapes found : " << to_string(num_of_shapes) << endl;
				cout << " Coordinates inside shapes : " << to_string(shapes_found[0].corner_coordinates.size()) << endl;
				cout << "coordinates value : " << to_string(shapes_found[0].corner_coordinates[0].x) << endl;

				Mat visualization;
				depthImage.convertTo(visualization, CV_8UC3);
				cvtColor(visualization, visualization, CV_GRAY2BGR);

				for (int i = 0; i < num_of_shapes; i++) {
					int num_of_corners = shapes_found[0].corner_coordinates.size();
					for (int j = 0; j < num_of_corners; j++) {
						circle(visualization, shapes_found[0].corner_coordinates[0], 30, Scalar(255, 0, 0), 5, 8, 0);
					}
				}

				
				
				// TODO : get the current pipeline device info
				// TODO : apply transformation according to camera position
				cur_pipeline_profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);


				// Display in a GUI
				imshow(depth_name.c_str(), visualization);
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
	}

	if (flags == (cv::EVENT_FLAG_LBUTTON + cv::EVENT_FLAG_SHIFTKEY))
	{
		std::cout << "Shift+Left clicked at : " << x << ", " << y << std::endl;
	}

	if (flags == (cv::EVENT_FLAG_RBUTTON))
	{
		std::cout << "Right mouse clicked at : " << x << ", " << y << std::endl;
	}

	if (flags == (cv::EVENT_FLAG_RBUTTON + cv::EVENT_FLAG_SHIFTKEY))
	{

		std::cout << "Shift+Right clicked at : " << x << ", " << y << std::endl;
	}
}