#include "camera_calibration.hpp"

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
		void set_candidate_image_points(vector<cv::Point2f> pts);

		vector<Point2f> candidateImagePoints;
		void setPatternPosition(float px, float py);

		vector<vector<Point2f>> imagePointsProjObj;
		vector<Point2f> candidate_image_points;
		vector<Point2f> get_candidate_image_points() { return candidate_image_points; }
		vector<vector<cv::Point3f>> get_object_points() { return objectPoints; }

		// object points in the world coordinate system
		vector<vector<cv::Point3f>> objectPoints;

		vector<Mat> get_board_rotations() { return boardRotations; }
		vector<Mat> get_board_translations() { return boardTranslations; }

		void calibrate();

		void setup_projector_parameters(Size _imageSize, string _outputFileName, Size _patternSize, float _squareSize, Pattern _patternType);

	private:
		// The cameraMatrix here is actually the projector intrinsics matrix. Since we are using inverse camera calibration, I'm leaving it named as cameraMatrix
		Mat cameraMatrix, distCoeffs;
		Size circlePatternSize;

		Size boardSize, imageSize;
		float squareSize;

		Pattern patternType;

		Point2f patternPosition;

		vector<Mat> boardRotations;
		vector<Mat> boardTranslations;

		string outputFileName;

#pragma region projector calibration methods

		static double computeReprojectionErrors(
			const vector<vector<Point3f> >& objectPoints,
			const vector<vector<Point2f> >& imagePoints,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			vector<float>& perViewErrors);

		// For the projector, the real world object points is estimated by the camera, and given as input
		static bool runCalibration(vector<vector<Point2f> > imagePoints,
			vector<vector<Point3f> > objectPoints,
			Size imageSize,  float aspectRatio,
			int flags, Mat& cameraMatrix, Mat& distCoeffs,
			vector<Mat>& rvecs, vector<Mat>& tvecs,
			vector<float>& reprojErrs,
			double& totalAvgErr);

		static void saveCameraParams(const string& filename,
			Size imageSize, float aspectRatio, int flags,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const vector<float>& reprojErrs,
			const vector<vector<Point2f> >& imagePoints,
			double totalAvgErr);

		static bool runAndSave(const string& outputFilename,
			const vector<vector<Point2f> >& imagePoints, vector<vector<Point3f> > objectPoints,
			Size imageSize,
			float aspectRatio, int flags, Mat& _cameraMatrix,
			Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

#pragma endregion

	};


}