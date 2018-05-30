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
Capture::Capture()
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
	//cvNamedWindow("Color", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	initializeSensor();
	cv::setMouseCallback("Depth", mouseCallback, this);
}

void Capture::finalize() {

}

void Capture::run()
{
	while (true) {
		update();
		draw();
		show();
		if (waitKey(1) == 99) {
			vector<Point2f> pts_src, pts_dest;
			pts_src.push_back(topLeftImage); pts_src.push_back(topRightImage); pts_src.push_back(bottomRightImage); pts_src.push_back(bottomLeftImage);
			pts_dest.push_back(topLeft); pts_dest.push_back(topRight); pts_dest.push_back(bottomRight); pts_dest.push_back(bottomLeft);
			calcHomographyMatrix(pts_src, pts_dest);
		}else if(waitKey(1) == 113) break;

	}
}

// Initialize Sensor
inline void Capture::initializeSensor()
{
	
	//Add desired streams to configuration
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//m_pipeline.start(cfg);
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
	//updateColor();
	updateDepth();
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
	imshow("Color", color);
}

inline void Capture::updateDepth()
{
	frames = m_pipeline.wait_for_frames();

	//Get each frame
	rs2::frame depth = frames.get_depth_frame();


	// Query frame size (width and height)
	const int w = depth.as<rs2::video_frame>().get_width();
	const int h = depth.as<rs2::video_frame>().get_height();

	cv::Mat depthImage = cv::Mat(h,
		w,
		CV_16U, (char*)depth.get_data());

	// 0-2000mm‚Ü‚Å‚Ìƒf[ƒ^‚ð0-255‚É‚·‚é
	depthImage.setTo(0, depthImage > 1500);
	depthImage.convertTo(depthImage, CV_8U, 255.0 / 1500);

	// Create OpenCV matrix of size (w,h) from the colorized depth data
	//Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

	// Display in a GUI
	imshow("Depth", depthImage);
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