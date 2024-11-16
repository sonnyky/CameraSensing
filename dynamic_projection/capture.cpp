#include "capture.hpp"

Tinker::capture::capture(const std::string& type, int index) : camera_type(type), camera_index(index)
{
	if (camera_type == "webcam") {
		cap.open(camera_index);

		if (!cap.isOpened()) {
			std::cerr << "Error: Could not open webcam with index " << camera_index << "\n";
		}
	}
}

Tinker::capture::~capture() {

}
