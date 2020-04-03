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


// Constructor
Capture::Capture():align_to_color(RS2_STREAM_COLOR)
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
	
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 5);
	initializeSensor();
	cv::setMouseCallback("Color", mouseCallback, this);
}

void Capture::finalize() {

}

void Capture::run()
{
	while (true) {
		update();
		
		if (waitKey(1) == 99) {
			trigger = true;
		}else if(waitKey(1) == 113) break;

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

// Initialize color image
inline void Capture::initializeColorImage()
{

}

// Update Data
void Capture::update()
{
	// Update Color
	updateColor();
	updateDepthWithPointCloud();
}

// Update Color
inline void Capture::updateColor()
{

	frames = m_pipeline.wait_for_frames();
	//Get each frame
	rs2::frame color_frame = frames.get_color_frame();

	// Creating OpenCV Matrix from a color image
	Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

	// Display in a GUI
	namedWindow("Display Image", WINDOW_AUTOSIZE);
	imshow("Display Image", color);
}

inline void Capture::updateDepthWithPointCloud() {

	frames = m_pipeline.wait_for_frames();
	//Get each frame
	rs2::frame depth_frame = frames.get_depth_frame();

	if (trigger) {
		rs2::frame filtered = depth_frame;
		// Note the concatenation of output/input frame to build up a chain
		filtered = dec_filter.process(filtered);
		points = pc.calculate(filtered);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points = points_to_pcl(points);

		// Remove far points
		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*cloud_filtered);

		mesh_converter.estimate(cloud_filtered);
		trigger = false;
	}
}


// Draw Data
void Capture::draw()
{

	// Draw Color
	drawColor();

}

// Draw Color
inline void Capture::drawColor()
{
	
}

// Show Data
void Capture::show()
{
	// Show Color
	showColor();

}

// Show Color
inline void Capture::showColor()
{
	
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

void Capture::estimate_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	mesh_converter.estimate(cloud);
}

pcl_ptr Capture::points_to_pcl(const rs2::points & points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}