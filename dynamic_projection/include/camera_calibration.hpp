#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace Tinker {
	using namespace std;
	using namespace cv;

	enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
	enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	class camera_calibration {

	public:

		camera_calibration();
		~camera_calibration();

		

	private:
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
			float grid_width, bool release_object,
			int flags, Mat& cameraMatrix, Mat& distCoeffs,
			vector<Mat>& rvecs, vector<Mat>& tvecs,
			vector<float>& reprojErrs,
			vector<Point3f>& newObjPoints,
			double& totalAvgErr);

		static void saveCameraParams(const string& filename,
			Size imageSize, Size boardSize,
			float squareSize, float aspectRatio, int flags,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const vector<float>& reprojErrs,
			const vector<vector<Point2f> >& imagePoints,
			const vector<Point3f>& newObjPoints,
			double totalAvgErr);

		static bool readStringList(const string& filename, vector<string>& l);

		static bool runAndSave(const string& outputFilename,
			const vector<vector<Point2f> >& imagePoints,
			Size imageSize, Size boardSize, Pattern patternType, float squareSize,
			float grid_width, bool release_object,
			float aspectRatio, int flags, Mat& cameraMatrix,
			Mat& distCoeffs, bool writeExtrinsics, bool writePoints, bool writeGrid);


	};
}