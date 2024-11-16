// This class provides utilities to capture from a camera.
#include <opencv2/opencv.hpp>

using namespace cv;

namespace Tinker {
	class capture {
	public:
		capture(const std::string& type, int index);
		~capture();

	private:
		std::string camera_type;
		int camera_index;
		VideoCapture cap;

	};
}