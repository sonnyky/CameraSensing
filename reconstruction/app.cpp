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
	setup_capture_parameters();
	initialize();
}

// Destructor
Capture::~Capture()
{
	// Finalize
	finalize();
}

void Capture::setup_capture_parameters()
{
	cout << "Setting up capture parameters:" << endl;
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file("parameters.xml");
	cout << "Load result: " << result.description() << endl;

	pugi::xpath_query query_filter_magnitude("/params/capture_params/filter_magnitude");
	rs_filter_magnitude = stoi(query_filter_magnitude.evaluate_string(doc));

	pugi::xpath_query query_time_diff("/params/capture_params/time_diff");
	time_diff = stoi(query_time_diff.evaluate_string(doc));

	pugi::xpath_query query_dist_min("/params/capture_params/dist_limit_min");
	dist_limit_min = stof(query_dist_min.evaluate_string(doc));

	pugi::xpath_query query_dist_max("/params/capture_params/dist_limit_max");
	dist_limit_max = stof(query_dist_max.evaluate_string(doc));

	pugi::xpath_query query_x_min("/params/capture_params/x_limit_min");
	x_limit_min = stof(query_x_min.evaluate_string(doc));

	pugi::xpath_query query_x_max("/params/capture_params/x_limit_max");
	x_limit_max = stof(query_x_max.evaluate_string(doc));

	pugi::xpath_query query_y_min("/params/capture_params/y_limit_min");
	y_limit_min = stof(query_y_min.evaluate_string(doc));

	pugi::xpath_query query_y_max("/params/capture_params/y_limit_max");
	y_limit_max = stof(query_y_max.evaluate_string(doc));

	pugi::xpath_query query_field_name("/params/capture_params/filter_field_name");
	filter_field_name = query_field_name.evaluate_string(doc);

	cout << "filter magnitude : " << rs_filter_magnitude << endl;
	cout << "dist_limit_min : " << dist_limit_min << endl;
	cout << "dist_limit_max : " << dist_limit_max << endl;
	cout << "filter_field_name : " << filter_field_name << endl;
}

void Capture::initialize() {
	
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, rs_filter_magnitude);
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
		}
		else if (waitKey(1) == 111) {
			// o key
			add_first_cloud();
		}
		else if (waitKey(1) == 112) {
			// p key
			setup_capture_parameters();
		}
		else if (waitKey(1) == 114) {
			// r key
			mesh_converter.setup_reconstruction_parameters();
		}
		else if (waitKey(1) == 116) {
			// t key
			add_second_cloud();
		}
		else if (waitKey(1) == 97) {
			// a key
			align_clouds();
		}
		else if (waitKey(1) == 98) {
			// b key
			cout << "b pressed!" << endl;
			mesh_converter.clear_aligned_clouds();
			prevTimeStamp = clock();
			cur_frame = 0;
			continuousScanning = true;
		}
		else if (waitKey(1) == 103) {
			// g key
			test_generate_mesh_aligned();
		}
		else if (waitKey(1) == 113) {
			break;
		}

	}
}

// Initialize Sensor
inline void Capture::initializeSensor()
{
	
	//Add desired streams to configuration
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	pipeline_profile profile_cap =  m_pipeline.start();
	auto device = profile_cap.get_device();
	std::vector<rs2::sensor> sensors = device.query_sensors();

	if (sensors.size() > 0) {
		sensors[0].set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
	}
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
	align_clouds_continuous();
}

// Update Color
inline void Capture::updateColor()
{

	frames = m_pipeline.wait_for_frames();
	//Get each frame
	rs2::frame color_frame = frames.get_color_frame();

	// Creating OpenCV Matrix from a color image
	Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	cvtColor(color, color, COLOR_BGR2RGB);
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

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::CropBox<pcl::PointXYZ> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(x_limit_min, y_limit_min, dist_limit_min, 1.0));
		boxFilter.setMax(Eigen::Vector4f(x_limit_max, y_limit_max, dist_limit_max, 1.0));
		boxFilter.setInputCloud(pcl_points);
		boxFilter.filter(*cloud_filtered);

		mesh_converter.estimate(cloud_filtered, "", false);
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

void Capture::estimate_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string fileName, bool useFile)
{
	mesh_converter.estimate(cloud, fileName, useFile);
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

void Capture::add_first_cloud()
{
	frames = m_pipeline.wait_for_frames();
	//Get each frame
	rs2::frame depth_frame = frames.get_depth_frame();

	rs2::frame filtered = depth_frame;
	// Note the concatenation of output/input frame to build up a chain
	filtered = dec_filter.process(filtered);
	points = pc.calculate(filtered);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points = points_to_pcl(points);

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(x_limit_min, y_limit_min, dist_limit_min, 1.0));
	boxFilter.setMax(Eigen::Vector4f(x_limit_max, y_limit_max, dist_limit_max, 1.0));
	boxFilter.setInputCloud(pcl_points);
	boxFilter.filter(*cloud_filtered);

	mesh_converter.add_to_cloud1(cloud_filtered);
		
}

void Capture::add_second_cloud()
{

	frames = m_pipeline.wait_for_frames();
	//Get each frame
	rs2::frame depth_frame = frames.get_depth_frame();

	rs2::frame filtered = depth_frame;
	// Note the concatenation of output/input frame to build up a chain
	filtered = dec_filter.process(filtered);
	points = pc.calculate(filtered);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points = points_to_pcl(points);

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(x_limit_min, y_limit_min, dist_limit_min, 1.0));
	boxFilter.setMax(Eigen::Vector4f(x_limit_max, y_limit_max, dist_limit_max, 1.0));
	boxFilter.setInputCloud(pcl_points);
	boxFilter.filter(*cloud_filtered);

	mesh_converter.add_to_cloud2(cloud_filtered);
}

void Capture::align_clouds()
{
	mesh_converter.align_and_save_clouds();
}

void Capture::test_generate_mesh_aligned()
{
	mesh_converter.generate_mesh_from_file();
}

void Capture::align_clouds_continuous()
{
	if (!continuousScanning) return;

	//Must align once first to prepare data
	clock_t now = clock();
	if (cur_frame <= max_frames && (now - prevTimeStamp > CLOCKS_PER_SEC * time_diff)) {
		cur_frame++;
		add_second_cloud();
		mesh_converter.continuous_scan_store_aligned_as_cloud1();
		prevTimeStamp = clock();
		cout << "done scanning" << now << endl;

	}
	if (cur_frame > max_frames) {
		continuousScanning = false;
		mesh_converter.save_and_generate_mesh();
		cout << "enough frames!" << endl;
	}
	
}
