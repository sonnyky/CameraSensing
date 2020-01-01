#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

using namespace cv;
using namespace std;

struct shape {
	vector<Point2i> corner_coordinates;
};

class shape_detector {
	
private :
	

	
public:
	// Constructor
	shape_detector();

	// Destructor
	~shape_detector();

	// Methods
	vector<shape> detect_shape(Mat frame);
	void test();

};