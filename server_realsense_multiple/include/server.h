#ifndef __APP__
#define __APP__

#undef UNICODE

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "proto/position.pb.h"

#include <ip/UdpSocket.h>
#include <osc/OscOutboundPacketStream.h>

#include <json/json.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

class Server {
private :
	int mode_; // mode = 0 is send data to remote client. mode = 1 is calibration mode

	// Network comms
	const char* osc_out_ip_ = "127.0.0.1";
	PCSTR incoming_port_ = "1101";
	int outgoing_port_ = 7777;
	int iResult_, iDataResult_;
	int messagelength_, chunk_;
	string data_;
	SOCKET ListenSocket_ = INVALID_SOCKET;
	SOCKET ClientSocket_ = INVALID_SOCKET;

	// Calibration
	int merged_image_width, merged_image_height;
	Mat merged_image;
	Mat homography;

	// reference points
	Point2f topLeft, topRight, bottomLeft, bottomRight;

	// reference points in image coordinates
	Point2f topLeftImage, topRightImage, bottomLeftImage, bottomRightImage;

	// list of candidate points for each of the reference points (image coordinates)
	vector<Point2f> topLefts, topRights, bottomLefts, bottomRights;
	int num_points_for_calib;

public:
	// Constructor
	Server();

	// Destructor
	~Server();

	// Networking functions
	void setup_osc_out_ip(const char* ip);
	void setup_socket();
	void process_data();
	void receive_data_length();
	int receive_data_chunk(string& test);
	void receive_data();
	void parse_data(string test);
	void close_socket();
	void cleanup();
	void osc_send(string str);

	// Calibration functions
	void SetHomography();
	void SetMode(int mode);
	void InitializeWindow();
	void ShowMergedImage(string data);
	void calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest);
	void setReferencePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY);
	void DetermineMergedImageDimensions();

	// Input functions
	static void mouseCallback(int event, int x, int y, int flags, void* userdata);
	inline void doMouseCallback(int event, int x, int y, int flags);

	// Utilities
	void PressEnterToContinue();
private :
	void initialize();
	void finalize();

	// TODO : Automatic calibration helper functions

	// TODO : Assigns a detected to point to one of the list of reference points
	void assign_to_group(Point2f point);

};

#endif // __APP__