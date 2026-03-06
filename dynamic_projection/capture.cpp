#include "capture.hpp"
using namespace std;

Tinker::capture::capture(const std::string& type, int index)
	: camera_type(type), camera_index(index)
{
	std::cout << "[capture ctor] this=" << this
		<< " &cap=" << &cap
		<< " index=" << camera_index
		<< " type=" << camera_type << "\n" << std::flush;

	if (camera_type == "webcam") {
		std::cout << "[capture ctor] before cap.open\n" << std::flush;
		bool ok = cap.open(camera_index, cv::CAP_DSHOW);
		std::cout << "[capture ctor] after cap.open ok=" << ok
			<< " isOpened=" << cap.isOpened() << "\n" << std::flush;
	}
}

Tinker::capture::~capture() {
	if (cap.isOpened()) {
		cap.release();
		std::cout << "Camera object with index " << camera_index << " is being destroyed." << std::endl;
	}
}

cv::Mat Tinker::capture::read()
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
