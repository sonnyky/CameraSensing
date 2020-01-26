#include "app.h"
#include "util.h"
#include <thread>
#include <chrono>

#include <numeric>

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

using namespace std;
using namespace rs2;


// Constructor
Capture::Capture():align_to_color(RS2_STREAM_COLOR)
{
	
}

// Destructor
Capture::~Capture()
{
	// Finalize
	finalize();
}

void Capture::initialize() {

	
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

	auto depthSensor = profile.get_device().query_sensors()[0];
	if (depthSensor.get_option(rs2_option::RS2_OPTION_VISUAL_PRESET)) {

		depthSensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
	}

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

vector<Mat> Capture::get_color_images()
{
	vector<Mat> color_images;
	poll_frames();
	auto total_number_of_streams = stream_count();

	if (total_number_of_streams == 0)
	{
		cout << "No streams available" << endl;
		return color_images;
	}

	std::lock_guard<std::mutex> lock(_mutex);
	int device_no = 0;
	for (auto&& view : _devices)
	{
		string ss_color_name = "Color";

		// For each device get its pipeline
		for (auto&& id_to_frame : view.second.color_frame)
		{
			// If the frame is available
			if (id_to_frame.second)
			{
				string color_name = ss_color_name.append(to_string(device_no));

				// Creating OpenCV Matrix from a color image
				Mat color(Size(640, 480), CV_8UC3, (void*)id_to_frame.second.get_data(), Mat::AUTO_STEP);
				Mat colorReversed;
				cvtColor(color, colorReversed, COLOR_BGR2RGB);

				color_images.push_back(colorReversed);
			}
		}
		device_no++;
	}

	return color_images;
}

vector<Mat> Capture::get_depth_data() {

	vector<Mat> depth_data;

	poll_frames();
	auto total_number_of_streams = stream_count();
	if (total_number_of_streams == 0)
	{
		cout << "No streams available" << endl;
		return depth_data;
	}

	std::lock_guard<std::mutex> lock(_mutex);

	int device_no = 0;

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
				depth_data.push_back(depthImage);
			}
		}
		device_no++;
	}
	return depth_data;
}

void Capture::set_alignment(int a)
{
	alignment = a;
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
			if (alignment == 1) {
				frameset = align_to_color.process(frameset);
			}
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
				Mat colorReversed;
				cvtColor(color, colorReversed, COLOR_BGR2RGB);
				// Display in a GUI
				imshow(color_name.c_str(), colorReversed);
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

				rs2::frame depth = id_to_frame.second.apply_filter(color_map);

				string depth_name = ss_depth_name.append(to_string(device_no));
				pipeline_profile cur_pipeline_profile = view.second.profile;

				// Query frame size (width and height)
				const int w = depth.as<rs2::video_frame>().get_width();
				const int h = depth.as<rs2::video_frame>().get_height();

				cv::Mat depthImage (h, w, CV_8UC3, (char*)depth.get_data(), Mat::AUTO_STEP);

				cur_pipeline_profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

				// Display in a GUI
				imshow(depth_name.c_str(), depthImage);
			}
		}
		device_no++;
	}

}