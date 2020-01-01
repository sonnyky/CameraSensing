#include "server.h"

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

using namespace std;

void save_error_to_file(string error) {
	ofstream myfile;
	auto end = std::chrono::system_clock::now();
	std::time_t end_time = std::chrono::system_clock::to_time_t(end);

	myfile.open("server_realsense_multiple_interrupted.txt");
	myfile << error;
	myfile.close();
}

Server::Server()
{
	initialize();
}
Server::~Server()
{
	// Finalize
	finalize();
}

void Server::initialize() {
	messagelength_ = 0;
	mode_ = 0;
	num_points_for_calib = 30;
	merged_image_height = 0;
	merged_image_width = 0;
	homography = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
}

void Server::finalize() {
	cleanup();
}

#pragma region sensor_networking

void Server::setup_osc_out_ip(const char* ip) {
	osc_out_ip_ = ip;
}

void Server::osc_send(string str) {

	// Transmit data to Unity
	if (!str.empty()) {
		UdpTransmitSocket transmitSocket(IpEndpointName(osc_out_ip_, outgoing_port_));
		zaboom::people_position elem;
		if (elem.IsInitialized()) {
			cout << "elem is initialized and ready to send!" << endl;
			elem.ParseFromString(str);
			transmitSocket.Send(str.data(), str.size());
		}
	}
}

void Server::setup_socket() {

	WSADATA wsaData;

	struct addrinfo *result = NULL;
	struct addrinfo hints;

	// Initialize Winsock
	iResult_ = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult_ != 0) {
		string error = "WSAStartup failed with error: " + to_string(iResult_);
		save_error_to_file(error);
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	iResult_ = getaddrinfo(NULL, incoming_port_, &hints, &result);
	if (iResult_ != 0) {
		string error = "getaddrinfo failed with error: " + to_string(iResult_);
		save_error_to_file(error);
		WSACleanup();
		return;
	}

	// Create a SOCKET for connecting to server
	ListenSocket_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket_ == INVALID_SOCKET) {
		string error = "socket failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		freeaddrinfo(result);
		WSACleanup();
		return;
	}

	// Setup the TCP listening socket
	iResult_ = ::bind(ListenSocket_, result->ai_addr, (int)result->ai_addrlen);
	if (iResult_ == SOCKET_ERROR) {
		string error = "bind failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		freeaddrinfo(result);
		closesocket(ListenSocket_);
		WSACleanup();
		return;
	}

	freeaddrinfo(result);

	iResult_ = listen(ListenSocket_, SOMAXCONN);
	if (iResult_ == SOCKET_ERROR) {
		string error = "listen failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		closesocket(ListenSocket_);
		WSACleanup();
		return;
	}

	// Accept a client socket
	ClientSocket_ = accept(ListenSocket_, NULL, NULL);
	if (ClientSocket_ == INVALID_SOCKET) {
		string error = "accept failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		closesocket(ListenSocket_);
		WSACleanup();
		return;
	}

	// No longer need server socket
	closesocket(ListenSocket_);
}

void Server::process_data() {
	do {
		receive_data_length();
		receive_data();
		if (data_.size() > 0) {

			// just for debugging
			parse_data(data_);

			// 0 : send to unity or 1 : calibrate
			switch (mode_) {
			case 0:
				osc_send(data_);
				break;
			case 1:
				// TODO : calibration mode
				ShowMergedImage(data_);

				// press c to calculate homography
				if (waitKey(1) == 99) {
					vector<Point2f> pts_src, pts_dest;
					pts_src.push_back(topLeftImage); pts_src.push_back(topRightImage); pts_src.push_back(bottomRightImage); pts_src.push_back(bottomLeftImage);
					pts_dest.push_back(topLeft); pts_dest.push_back(topRight); pts_dest.push_back(bottomRight); pts_dest.push_back(bottomLeft);
					calcHomographyMatrix(pts_src, pts_dest);
				}
				break;
			default:
				break;
			}
		}
		else {
			cout << "test string empty" << endl;
		}
	} while (iResult_ > 0);
}

void Server::receive_data_length() {
		//std::cout << "Messagelength before recv : " << to_string(messagelength_) << endl;
		iResult_ = recv(ClientSocket_, (char *)&messagelength_, sizeof(int), 0);
		if (iResult_ > 0) {
			// Normal operation
		}
		else if (iResult_ == 0) {
			string error = "Message length is zero bytes";
			save_error_to_file(error);
		}
		else {
			string error = "[receive_data_length] iResult recv failed with error: " + to_string(WSAGetLastError());
			save_error_to_file(error);
			return;
		}
}

void Server::receive_data() {
	string rcv;
	chunk_ = 0;
	data_.clear();

	do {
		//std::cout << "chunk is : " << chunk_ << "and total message length is : " << messagelength_ << endl;
		if(receive_data_chunk(data_) > 0) break;
		if (chunk_ >= messagelength_)
			break;
	} while (chunk_ <= messagelength_);
}

void Server::parse_data(string test) {
	zaboom::people_position last_elem;
	last_elem.ParseFromString(test);
	cout << "image position x " << to_string(last_elem.x(0)) << ", " << to_string(last_elem.y(0)) << endl;
}

int Server::receive_data_chunk(string& test) {

	vector<char> recv_char(messagelength_);
	iDataResult_ = recv(ClientSocket_, recv_char.data(), messagelength_, 0);

	if (iDataResult_ > 0) {
		printf("iDataResult Bytes received: %d\n", iDataResult_);

		test.append(recv_char.begin(), recv_char.end());
		chunk_ += iDataResult_;
	}
	else if (iDataResult_ == 0) {
		string error = "iDataResult Connection not receiving data ";
		save_error_to_file(error);
		return 1;
	}
	else {
		string error = "[receive_data_chunk] iDataResult recv failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		return 1;
	}
	return 0;
}

void Server::close_socket() {
	iResult_ = shutdown(ClientSocket_, SD_SEND);
	if (iResult_ == SOCKET_ERROR) {
		string error = "shutdown failed with error: " + to_string(WSAGetLastError());
		save_error_to_file(error);
		closesocket(ClientSocket_);
		WSACleanup();
		return;
	}
}

void Server::cleanup() {
	closesocket(ClientSocket_);
	WSACleanup();
}

#pragma endregion getting sensor data through tcp socket

#pragma region calibration

void Server::DetermineMergedImageDimensions() {
	std::ifstream calib_file("calib_file.json", std::ifstream::binary);
	Json::Value calib_settings;
	calib_file >> calib_settings;

	int highest_xpos = -1, highest_ypos = -1;

	// Determine the highest value for x and y position matrix ids
	for (unsigned int i = 0; i < calib_settings["devices"].size(); i++) {
		if (calib_settings["devices"][i]["pos_x"].asInt() > highest_xpos) highest_xpos = calib_settings["devices"][i]["pos_x"].asInt();
		if (calib_settings["devices"][i]["pos_y"].asInt() > highest_ypos) highest_ypos = calib_settings["devices"][i]["pos_y"].asInt();
	}

	// ASSUMPTION : All images have the same dimensions
	merged_image_width = 1280 + (highest_ypos * 1280);
	merged_image_height = 720 + (highest_xpos * 720);

	cout << " highest_xpos: " << highest_xpos << endl;
	cout << " highest_ypos: " << highest_ypos << endl;

	cout << "Merged image width : " << merged_image_width << endl;
	cout << "Merged image height : " << merged_image_height << endl;

	InitializeWindow();
}

void Server::InitializeWindow() {
	cvNamedWindow("Merged", CV_WINDOW_NORMAL);
	cv::setMouseCallback("Merged", mouseCallback, this);
}

void Server::ShowMergedImage(string test) {
	zaboom::people_position last_elem;
	last_elem.ParseFromString(test);

	// must have more than four points as input to homography calculations.
//	if (last_elem.x_size() == last_elem.y_size() && last_elem.x_size() >= 4) {
		cv::Mat merged(merged_image_height, merged_image_width, CV_8UC3);
		merged.setTo(0);

		// Draw red circles to check image size
		circle(merged, Point(2560, 720), 20, Scalar(0, 0, 255), -1, 8);

		// Draw circles on the reference points, only this time it is adjusted to the merged image coordinates.
		for (int i = 0; i < last_elem.x_size(); i++ ) {
			circle(merged, Point(last_elem.x(i), last_elem.y(i)), 20, Scalar(255, 0, 0), 2, 8);
		}

		// Opencv window will go out of screen, so we need to resize to fit screen or set to CV_WINDOW_NORMAL
		//resize(merged, merged, Size(1280, 720));

		imshow("Merged", merged);
//	}
}

// TODO : automatically assign points to a group of reference points, e.g. top_lefts
void Server::assign_to_group(Point2f point) {
	// TODO : assign points to group
}

#pragma endregion calibration mode

#pragma region homography

void Server::SetHomography() {
	//ASSUMPTION : homography file is in the same folder
	FileStorage fs("homography.xml", FileStorage::READ);
	fs["Homography"] >> homography;
}

void Server::setReferencePoints(int topLeftX, int topLeftY, int topRightX, int topRightY, int bottomLeftX, int bottomLeftY, int bottomRightX, int bottomRightY) {
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

void Server::calcHomographyMatrix(vector<Point2f> pts_src, vector<Point2f> pts_dest) {
	printf("Calculating homography...\n Please redo clicking if the results are not as expected.\n Matrix will be output as text file.");
	Mat h = findHomography(pts_src, pts_dest);
	FileStorage file("homography.xml", 1, "UTF-8");
	file <<"Homography" << h;
}

#pragma endregion homography calculations and file output

#pragma region utilities

void Server::SetMode(int mode) {
	mode_ = mode;
}

void Server::PressEnterToContinue()
{
	std::cout << "Press ENTER to continue... " << flush;
	std::cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
}

void Server::mouseCallback(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN)
	{
		Server* self = static_cast<Server*>(userdata);
		self->doMouseCallback(event, x, y, flags);
	}
}

void Server::doMouseCallback(int event, int x, int y, int flags) {
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

#pragma endregion helper functions