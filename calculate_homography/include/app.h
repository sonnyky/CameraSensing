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
private :

	rs2::pipeline m_pipeline;
	rs2::config cfg;
	rs2::frameset frames;
	rs2::colorizer color_map;

	// Variables to compute homography
	Point2f topLeft, topRight, bottomLeft, bottomRight;
	Point2f topLeftImage, topRightImage, bottomLeftImage, bottomRightImage;

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

	// Calibration functions
	void calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest);
	void setRangePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY);

private :
	void initialize();
	void finalize();

	// Sensor initializations
	inline void initializeSensor();
	inline void initializeColorImage();

	// Drawing functions
	void update();
	inline void updateColor();
	inline void updateDepth();
	void draw();
	inline void drawColor();
	void show();
	inline void showColor();

	// Input functions
	static void mouseCallback(int event, int x, int y, int flags, void* userdata);
	inline void doMouseCallback(int event, int x, int y, int flags);
};

#endif // __APP__