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
				color_depth.at<Vec3b>(i, j) = Vec3b(255 - f, 0,f);
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
	setup_capture_parameters();

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
	
		if (waitKey(1) == 113) {
			break;
		}
		else if (waitKey(1) == 99) {
			SaveSingleCloud();
		}
		else if (waitKey(1) == 115 && !save_pose_and_cloud) {
			save_pose_and_cloud = true;
			SavePoseCloud();
		}
		else if (waitKey(1) == 97 && !align_and_reconstruct) {
			align_and_reconstruct = true;
			camera_position_.ClearAlignedCloud();
			AlignAndReconstruct();
		}
	}
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

// Initialize Sensor
inline void Capture::initializeSensor()
{

	camera_position_.Initialize(string("camera_params.xml"), Size(7,4));
	
	//Add desired streams to configuration
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	m_pipeline.start();

	
	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = m_pipeline.wait_for_frames();
	}

}
void Capture::TrackCameraPosition()
{
	if (!current_color_frame) return;
	camera_position_.DetectChessboard(current_color_image);
}
// Update Data
void Capture::update()
{
	poll_frames();
	if (!stream_exists()) {
		cout << "No streams" << endl;
	}
	updateColor();

	TrackCameraPosition();

	updateDepth();
}

// Update Color
void Capture::updateColor()
{
	if (!current_color_frame) return;

	// Creating OpenCV Matrix from a color image
	Mat color(Size(640, 480), CV_8UC3, (void*)current_color_frame.get_data(), Mat::AUTO_STEP);
	color.copyTo(current_color_image);

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


void Capture::SaveSingleCloud() {
	cout << "save single shot" << endl;
	camera_position_.SaveSingleShotCloud(GeneratePointCloud());
}

pcl_ptr Capture::GeneratePointCloud() {
	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	if (!current_depth_frame) return cloud_filtered;
	rs2::frame filtered = current_depth_frame;
	// Note the concatenation of output/input frame to build up a chain
	points = pc.calculate(filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points = points_to_pcl(points);

	
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(x_limit_min, y_limit_min, dist_limit_min, 1.0));
	boxFilter.setMax(Eigen::Vector4f(x_limit_max, y_limit_max, dist_limit_max, 1.0));
	boxFilter.setInputCloud(pcl_points);
	boxFilter.filter(*cloud_filtered);
	return cloud_filtered;
}

void Capture::SavePoseCloud()
{
	camera_position_.SaveCurrentPoseAndCloud(GeneratePointCloud());
	save_pose_and_cloud = false;
}

void Capture::AlignAndReconstruct()
{
	camera_position_.AlignAndReconstructClouds();
	align_and_reconstruct = false;
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

