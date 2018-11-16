#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <comutil.h>
#include <iostream>
#include <cstdio>
#include <ctime>

#include <wtypes.h>
#include <comdef.h> 
#include <string>
#include <string.h>
#include <tchar.h>
#include <stdio.h>
#include "atlbase.h"
#include "atlwin.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <librealsense2/rs.hpp> 

#include <wrl/client.h>
using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;


class Capture {
	struct view_port
	{
		std::map<int, rs2::frame> color_frame;
		std::map<int, rs2::frame> depth_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		string device_number;
	};
private :
	rs2::config cfg;
	rs2::frameset frames;
	rs2::colorizer color_map;

	// Active distance threshold
	int low_dist_min, low_dist_max, high_dist_min, high_dist_max;

	// Blob detection
	SimpleBlobDetector::Params params;
	Ptr<SimpleBlobDetector> d;
	std::vector<KeyPoint> keypoints;
	std::vector<Vec2i> people_pos;

	// Variables to compute homography
	Point2f topLeft, topRight, bottomLeft, bottomRight;
	Point2f topLeftImage, topRightImage, bottomLeftImage, bottomRightImage;

	std::mutex _mutex;
	std::map<std::string, view_port> _devices;

	// Variables to erode and dilate image to improve detection
	int erosion_size;
	Mat element;

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

	// Calibration functions
	void calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest);
	void setRangePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY);

	void Capture::enable_device(rs2::device dev);
	void Capture::remove_devices(const rs2::event_information& info);

	size_t Capture::device_count();

	// Setting capture and image processing parameters
	void set_detection_params(int lowDistMin, int lowDistMax, int highDistMin, int highDistMax, int minBlobArea, int maxBlobArea, int erosionSize);
	void set_default_params();

private :
	void initialize();
	void finalize();

	// Drawing functions
	void poll_frames();
	int Capture::stream_count();
	void update();
	inline void updateColor();
	inline void updateDepth();

	// Input functions
	static void mouseCallback(int event, int x, int y, int flags, void* userdata);
	inline void doMouseCallback(int event, int x, int y, int flags);
};

#endif // __APP__