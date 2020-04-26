#include "app.h"
#include "util.h"
#include <thread>
#include <chrono>

#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp> 

using namespace std;
using namespace rs2;


inline void make_depth_histogram(const Mat &depth, Mat &color_depth) {
	color_depth = Mat(depth.size(), CV_8UC3);
	int width = depth.cols, height = depth.rows;

	static uint32_t histogram[0x10000];
	memset(histogram, 0, sizeof(histogram));

	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			++histogram[depth.at<ushort>(i, j)];
		}
	}

	for (int i = 2; i < 0x10000; ++i) histogram[i] += histogram[i - 1]; // Build a cumulative histogram for the indices in [1,0xFFFF]

	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			if (uint16_t d = depth.at<ushort>(i, j)) {
				int f = histogram[d] * 255 / histogram[0xFFFF]; // 0-255 based on histogram location
				color_depth.at<Vec3b>(i, j) = Vec3b(f, 0, 255 - f);
			}
			else {
				color_depth.at<Vec3b>(i, j) = Vec3b(0, 5, 20);
			}
		}
	}
}


// Constructor
Capture::Capture() :align_to_color(RS2_STREAM_COLOR),
					current_depth_frame(nullptr)
{
	// Initialize
	initialize();
}

// Destructor
Capture::~Capture()
{
	// Finalize
	finalize();
}

void Capture::initialize() {
	
	initializeSensor();
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

// Initialize Sensor
inline void Capture::initializeSensor()
{
	
	//Add desired streams to configuration
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	m_pipeline.start();

	
	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = m_pipeline.wait_for_frames();
	}

}
// Update Data
void Capture::update()
{
	poll_frames();
	if (!stream_exists()) {
		cout << "No streams" << endl;
	}
	updateColor();
	updateDepth();
}

// Update Color
void Capture::updateColor()
{
	rs2::frame color_frame = frames.get_color_frame();

	if (!color_frame) return;

	// Creating OpenCV Matrix from a color image
	Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

	// Display in a GUI
	namedWindow("Display Image", WINDOW_AUTOSIZE);
	imshow("Display Image", color);
}

void Capture::updateDepth(){
	

	if (!current_depth_frame) return;

	// Query frame size (width and height)
	const int w = current_depth_frame.as<rs2::video_frame>().get_width();
	const int h = current_depth_frame.as<rs2::video_frame>().get_height();

	cv::Mat depthImage = cv::Mat(h, w, CV_16U, (char*)current_depth_frame.get_data());

	// Create a color depth
	Mat color_depth;
	make_depth_histogram(depthImage, color_depth);

	// Create a normalized depth
	double min, max;
	minMaxLoc(depthImage, &min, &max);
	Mat depth_normalized;
	double alpha = 255.0 / (max - min);
	depthImage.convertTo(depth_normalized, CV_8U, alpha, -min * alpha);

	imshow("Depth", color_depth);
}


void Capture::poll_frames()
{
	std::lock_guard<std::mutex> lock(_mutex);
	
	if (m_pipeline.poll_for_frames(&frames))
	{
		
		frames = align_to_color.process(frames);
		//frames.apply_filter(color_map);
		
		current_color_frame = frames.get_color_frame();
		current_depth_frame = frames.get_depth_frame();
	}
}

bool Capture::stream_exists()
{
	if(!frames.get_color_frame() || !frames.get_depth_frame()) return false;
	return true;
}


void Capture::save_depth_and_color_frameset()
{
	
}
