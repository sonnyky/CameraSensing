#pragma once

#include <iostream>
#include <chrono>
#include <thread>
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
	

	enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
	enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	class camera_calibration {

	public:

		camera_calibration();
		~camera_calibration();

		void setup_parameters(
			Size boardSize_, 
			Size imageSize_,
			string pattern_,
			float squareSize_, 
			float aspectRatio_,
			int nFrames_,
			int delay_,
			int mode_,
			bool writePoints_,
			bool writeExtrinsics_,
			int cameraId_,
			string outputFileName_);

		bool calibrate(Mat image_);

		void set_to_calibration_mode();

		void undistort_image(Mat image);

		// Setup the points on the real world coordinate system for computing the board pose as seen by the camera
		void setup_candidate_object_points();

		void load(string camera_config);

		bool find_board(Mat img);
		void compute_candidate_board_pose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans);

		// Back project points in the image coordinates to real world (board) coordinates using the extrinsics matrices
		bool back_project(const Mat& boardRot64,
			const Mat& boardTrans64,
			const vector<Point2f>& imgPt,
			vector<Point3f>& worldPt);

		// candidate object points in the world coordinate system as seen from the real world (board) coordinates
		vector<cv::Point3f> candidateObjectPts;

		// object points in the world coordinate system
		vector<vector<cv::Point3f>> objectPoints;

		vector<vector<Point2f>> imagePointsCamObj;

		vector<cv::Point3f> get_candidate_object_points() { return candidateObjectPts; }
		vector<vector<cv::Point3f>> get_object_points() { return objectPoints; }

		vector<Mat> get_board_rotations() { return boardRotations; }
		vector<Mat> get_board_translations() { return boardTranslations; }
		vector<Mat> boardRotations;
		vector<Mat> boardTranslations;

		Size get_board_size() { return boardSize; }

		Mat get_camera_matrix() { return cameraMatrix; }
		Mat get_dist_coeffs() { return distCoeffs; }

		void load_camera_matrix(string fileName);

		Size get_image_size() { return imageSize; };

		vector<Point2f> get_detected_board_points() { return detected_board_points; };

	private:

		vector<Point2f> detected_board_points;

		

#pragma region camera calibration variables
		Size boardSize, imageSize;
		float patternLengthInRealUnits, aspectRatio;
		Mat cameraMatrix, distCoeffs;
		string outputFilename;

		int i, nframes;
		bool writeExtrinsics, writePoints;
		bool undistortImage = false;
		int flags = 0;

		bool flipVertical;
		bool showUndistorted;
		bool videofile;
		int delay;
		clock_t prevTimestamp = 0;
		int calibrationStatus = DETECTION;
		int cameraId = 0;
		vector<string> imageList;
		String pattern = "chessboard";

		bool camera_is_calibrated;
		vector<vector<Point2f>> imagePoints;

		vector<Point2f> pointbuf;
		bool found;
		clock_t previous_timestamp = 0;
#pragma endregion

#pragma region camera calibration methods

		static double computeReprojectionErrors(
			const vector<vector<Point3f> >& objectPoints,
			const vector<vector<Point2f> >& imagePoints,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			vector<float>& perViewErrors);

		static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD);

		static bool runCalibration(vector<vector<Point2f> > imagePoints,
			Size imageSize, Size boardSize, Pattern patternType,
			float squareSize, float aspectRatio,
			int flags, Mat& cameraMatrix, Mat& distCoeffs,
			vector<Mat>& rvecs, vector<Mat>& tvecs,
			vector<float>& reprojErrs,
			double& totalAvgErr);

		static void saveCameraParams(const string& filename,
			Size imageSize, Size boardSize,
			float squareSize, float aspectRatio, int flags,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const vector<float>& reprojErrs,
			const vector<vector<Point2f> >& imagePoints,
			double totalAvgErr);

		static bool runAndSave(const string& outputFilename,
			const vector<vector<Point2f> >& imagePoints,
			Size imageSize, Size boardSize, Pattern patternType, float squareSize,
			float aspectRatio, int flags, Mat& _cameraMatrix,
			Mat& distCoeffs, bool writeExtrinsics, bool writePoints);
	};
#pragma endregion
}