#include "shape_detector.h"

#include <numeric>

using namespace std;


// Constructor
shape_detector::shape_detector()
{

}

// Destructor
shape_detector::~shape_detector()
{
	
}

vector<shape> shape_detector::detect_shape(Mat frame) {

	vector<shape> detected_shapes;

	// Create default point to indicate no shapes found.
	Point default_point;
	default_point.x = -1;
	default_point.y = -1;

	Mat bw;
	if (frame.channels() > 1) {

		Mat gray;

		// change this !!
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		
		Canny(gray, bw, 800, 850, 5, true);
	}
	else {
		Canny(frame, bw, 800, 850, 5, true);
	}

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(bw.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size() < 1) {
		shape default_shape;
		default_shape.corner_coordinates.push_back(default_point);
		detected_shapes.push_back(default_shape);
		return detected_shapes;
	}
	
	for (int i = 0; i < contours.size(); i++) {
		double contourLength = arcLength(contours[i], true);
		double area = contourArea(contours[i]);

		// Reject small contours
		if (contourLength < 100 || area < 1000) continue;
		shape found_shape;
		vector<Point> approx;

		approxPolyDP(contours[i], approx, contourLength * 0.04, true);

		if (approx.size() > 2 && approx.size() < 7) {
			
			found_shape.corner_coordinates = approx;
			detected_shapes.push_back(found_shape);
		}
	}

	if (detected_shapes.size() < 1) {
		shape default_shape;
		default_shape.corner_coordinates.push_back(default_point);
		detected_shapes.push_back(default_shape);
		return detected_shapes;
	}

	return detected_shapes;
}

void shape_detector::test() {
	cout << " This is a test for the shape detector class" << endl;
}
