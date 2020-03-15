#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace std;
using namespace cv;

namespace Tinker {
	class projector_calibration {
	public:
		projector_calibration();
		~projector_calibration();

		void load(string projector_config);

		Mat get_dist_coeffs() { return distCoeffs; }
		Mat get_camera_matrix() { return cameraMatrix; }
		Size get_circle_pattern_size() { return circlePatternSize; }

		void set_static_candidate_image_points();
		vector<Point2f> candidateImagePoints;
		void setPatternPosition(float px, float py);

		vector<vector<Point2f>> imagePointsProjObj;
		vector<Point2f> candidate_image_points;
		vector<Point2f> get_candidate_image_points() { return candidate_image_points; }
		vector<vector<cv::Point3f>> get_object_points() { return objectPoints; }

		// object points in the world coordinate system
		vector<vector<cv::Point3f>> objectPoints;

	private:
		Mat cameraMatrix, distCoeffs;
		Size circlePatternSize;

		Size boardSize;
		float squareSize;

		Point2f patternPosition;
	};


}