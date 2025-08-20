#include "capture.hpp"
using namespace std;

Tinker::capture::capture(const std::string& type, int index) : camera_type(type), camera_index(index)
{
	if (camera_type == "webcam") {
		
		cap.open(camera_index);
		cout << "cap open " << endl;
		if (!cap.isOpened()) {
			std::cerr << "Error: Could not open webcam with index " << camera_index << "\n";
		}
		else {
			cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		}
	}
}

Tinker::capture::~capture() {
	if (cap.isOpened()) {
		cap.release();
		std::cout << "Camera object with index " << camera_index << " is being destroyed." << std::endl;
	}
}

Mat Tinker::capture::read()
{
	if (cap.isOpened()) {
		cap >> frame_image; // Capture a frame
		if (frame_image.empty()) {
			std::cerr << "Warning: Captured frame is empty\n";
		}
	}
	else {
		std::cerr << "Error: VideoCapture is not opened\n";
	}

	// Return a copy to prevent external modification
	return frame_image.clone();
}
