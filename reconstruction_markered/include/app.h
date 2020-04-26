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
#include<mutex>
#include <wrl/client.h>
using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;


class Capture {
	struct single_frame
	{
		rs2::frame color_frame;
		rs2::frame depth_frame;
	};

	struct single_frame_images
	{
		Mat color_image;
		Mat depth_image;
	};

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

private :
	void initialize();
	void finalize();

	// Sensor initializations
	inline void initializeSensor();

	void poll_frames();

	rs2::pipeline m_pipeline;
	rs2::config cfg;
	rs2::frameset frames;
	std::mutex _mutex;

	vector<single_frame> views;

	// capture a single frame of depth and color frames to synchronize data
	void save_depth_and_color_frameset();

	bool stream_exists();
	rs2::frame current_color_frame;
	rs2::depth_frame current_depth_frame;

	rs2::colorizer color_map;

	// Drawing functions
	void update();
	void updateColor();
	void updateDepth();
	rs2::align align_to_color;
};

#endif // __APP__