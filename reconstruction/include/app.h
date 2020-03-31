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

#include "pcl_to_mesh.h"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;
using namespace Tinker;

class Capture {
private :

	rs2::pipeline m_pipeline;
	rs2::config cfg;
	rs2::frameset frames;

	// Variables to compute homography
	Point2f topLeft, topRight, bottomLeft, bottomRight;
	Point2f topLeftImage, topRightImage, bottomLeftImage, bottomRightImage;

	pcl_to_mesh mesh_converter;

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

	void estimate_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	bool trigger = false;

	pcl_ptr points_to_pcl(const rs2::points& points);
	rs2::align align_to_color;
	rs2::decimation_filter dec_filter;

private :
	void initialize();
	void finalize();

	// Sensor initializations
	inline void initializeSensor();
	inline void initializeColorImage();

	// Drawing functions
	void update();
	inline void updateColor();
	inline void updateDepthWithPointCloud();
	void draw();
	inline void drawColor();
	void show();
	inline void showColor();

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Input functions
	static void mouseCallback(int event, int x, int y, int flags, void* userdata);
	inline void doMouseCallback(int event, int x, int y, int flags);
};

#endif // __APP__