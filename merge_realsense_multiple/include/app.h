#ifndef __APP__
#define __APP__

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <WinSock2.h>
#include <Windows.h>
#include <ws2tcpip.h>
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
#include <json/json.h>
#include "proto/position.pb.h"
#include <stdexcept>

#include <wrl/client.h>
using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;

#pragma comment (lib, "Ws2_32.lib")

class Capture {
	struct view_port
	{
		std::map<int, rs2::frame> color_frame;
		std::map<int, rs2::frame> depth_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		string device_number;
		Point2f position;
		Point2f overlap;
		Point2f detection_limit;
		vector<Point2i> obstacles;
	};
private :
	rs2::config cfg;
	rs2::frameset frames;
	rs2::colorizer color_map;

	// Network comms
	const char* host_name = "127.0.0.1";
	int port = 1101;
	int iResult;
	SOCKET ConnectSocket = INVALID_SOCKET;
	char *sendbuf = "this is a test";
	int buf_length;
	char recvbuf[512];
	int recvbuflen = 512;

	// check if position adjustment is done by client
	int positionAdjustByClient;

	// vector of serialized protobuf objects
	vector<string> serialized_positions;

	// Active distance threshold
	int distanceThreshold;

	// Blob detection
	SimpleBlobDetector::Params params;
	int low_dist_min, low_dist_max, high_dist_min, high_dist_max;
	int frame_count;

	Ptr<SimpleBlobDetector> d;
	std::vector<KeyPoint> keypoints;
	std::vector<Vec2i> people_pos;

	// Variables to erode and dilate image to improve detection
	int erosion_size;
	Mat element;
	void init_morph_element();

	// Variable to adjust point to be closer to center point
	double maxRadius;
	int adjustment_;

	// Homography variables
	Point2f topLeft, topRight, bottomLeft, bottomRight;
	Point2f topLeftImage, topRightImage, bottomLeftImage, bottomRightImage;
	Mat homography;

	std::mutex _mutex;
	std::map<std::string, view_port> _devices;

	vector<zaboom::people_position> positions;

public:
	// Constructor
	Capture();

	// Destructor
	~Capture();
	void run();

	// Calibration functions
	void set_homography_matrix(Mat mat);
	void read_homography_file();
	void calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest);
	void setRangePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY);

	void Capture::enable_device(rs2::device dev);
	void Capture::remove_devices(const rs2::event_information& info);

	size_t Capture::device_count();

	// Setting capture and image processing parameters
	void set_detection_params(int lowDistMin, int lowDistMax, int maxDistMin, int maxDistMax, int minBlobArea, int maxBlobArea, int erosionSize, int adjustment);
	void set_default_params();
	void set_adjustment_mode(int mode);

	// Networking functions
	void set_server_ip(const char * host);
	void setup_socket();
	void send_length_to_socket(int buf_to_send);
	bool receive_length_confirmation();
	void close_socket();
	bool can_send_data;
	void send_data(const char * data, int size);

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