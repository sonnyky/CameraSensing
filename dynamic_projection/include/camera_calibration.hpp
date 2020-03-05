#pragma once

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

		void calibrate(Mat image_);

		void set_to_calibration_mode();

		void undistort_image(Mat image);

	private:

		Size boardSize, imageSize;
		float squareSize, aspectRatio;
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
		int mode = DETECTION;
		int cameraId = 0;
		vector<vector<Point2f> > imagePoints;
		vector<string> imageList;
		String pattern = "chessboard";

		vector<Point2f> pointbuf;
		bool found;
		clock_t previous_timestamp = 0;

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
			float aspectRatio, int flags, Mat& cameraMatrix,
			Mat& distCoeffs, bool writeExtrinsics, bool writePoints);
	};
}