#include "include\calibration.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include "flags.hpp"

Tinker::calibration::calibration() :
	min_images_diff(4.0),
	min_elapsed_time(2.0),
	diff_mean(0.0),
	elapsed_time(0.0)
{
	camera_is_calibrated = false;
}

Tinker::calibration::~calibration()
{
}

void Tinker::calibration::setup_camera_calibration_parameters(Size boardSize_, Size imageSize_, string pattern_, float squareSize_, float aspectRatio_, int nFrames_, int delay_, int mode_, bool writePoints_, bool writeExtrinsics_, int cameraId_, string outputFileName_)
{
	camera_calibrator.setup_parameters(
		boardSize_,
		imageSize_,
		pattern_,
		squareSize_,
		aspectRatio_,
		nFrames_,
		delay_,
		mode_,
		writePoints_,
		writeExtrinsics_,
		cameraId_,
		outputFileName_
	);
	last_frame_time = system_clock::now();
	camera_calibrator.setup_candidate_object_points();

	// check for previous calibration files with the same file name
	struct stat buffer;
	bool fileFound = stat(outputFileName_.c_str(), &buffer) == 0;
	cout << "Camera calibration file exists : " << fileFound << endl;
	if (fileFound) {
		camera_is_calibrated = true;
		FileStorage fs(outputFileName_, FileStorage::READ);
		fs["camera_matrix"] >> camera_matrix;
	}
}

void Tinker::calibration::setup_projector_calibration_parameters(
	Size _imageSize, 
	string _outputFileName, 
	Size _patternSize, 
	float _squareSize, 
	int _nFramesBeforeDynamicProjectorCalib, 
	int _nFramesDynamicProjectorCalib,
	Pattern _patternType, 
	float px, 
	float py
)
{
	projector_calibrator.setup_projector_parameters(_imageSize, _outputFileName, _patternSize, _squareSize, _nFramesBeforeDynamicProjectorCalib, _patternType, px, py);
	required_dynamic_projector_samples = static_cast<uint64_t>(_nFramesDynamicProjectorCalib);
}

void Tinker::calibration::set_projector_static_image_points()
{
	projector_calibrator.set_static_candidate_image_points();
}

bool Tinker::calibration::calibrate_camera(Mat image)
{
	if (!accept_new_frame(image)) {
		return false;
	}
	else {
		return camera_calibrator.calibrate(image);
	}
}

void Tinker::calibration::switch_to_calibration_mode()
{
	camera_calibrator.set_to_calibration_mode();
}

void Tinker::calibration::load(string cameraConfig, string projectorConfig, string extrinsicsConfig)
{
	camera_calibrator.load(cameraConfig);
	projector_calibrator.load(projectorConfig);
	loadExtrinsics(extrinsicsConfig);
}

bool Tinker::calibration::accept_new_frame(cv::Mat camMat)
{
	if (prev_camera_frame.empty()) {
		camMat.copyTo(prev_camera_frame);
		last_frame_time = std::chrono::system_clock::now();
		std::cout << "first frame" << std::endl;
		return false;  // Do not accept the first frame
	}

	cv::Mat diffMat;
	cv::absdiff(prev_camera_frame, camMat, diffMat);

	cv::Scalar m = mean(diffMat);  // (meanB, meanG, meanR)
	double frameDiffMean = (m[0] + m[1] + m[2]) / 3.0;

	using namespace std::chrono;
	std::chrono::time_point<system_clock> latest_frame_time = system_clock::now();
	duration<double> elapsed_seconds = latest_frame_time - last_frame_time;

	double elapsed = elapsed_seconds.count();

	if ((elapsed > min_elapsed_time) && (frameDiffMean > min_images_diff)) {
		camMat.copyTo(prev_camera_frame);
		last_frame_time = latest_frame_time;
		diff_mean = frameDiffMean;
		elapsed_time = elapsed;
		return true;
	}

	return false;
}

void Tinker::calibration::loadExtrinsics(string filename, bool absolute)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Rotation_Vector"] >> rotCamToProj;
	fs["Translation_Vector"] >> transCamToProj;
}

// obtains points in the projector image coordinates that correspond to points in real world coordinates. This can be used as a measure of accuracy of projected points?
vector<Point2f> Tinker::calibration::get_projected(const vector<Point3f>& pts, const cv::Mat & rotObjToCam, const cv::Mat & transObjToCam)
{
	cv::Mat rotObjToProj, transObjToProj;

	cv::composeRT(rotObjToCam, transObjToCam,
		rotCamToProj, transCamToProj,
		rotObjToProj, transObjToProj);

	vector<Point2f> out;
	projectPoints(Mat(pts),
		rotObjToProj, transObjToProj,
		projector_calibrator.get_camera_matrix(),
		projector_calibrator.get_dist_coeffs(),
		out);
	return out;
}

bool Tinker::calibration::set_dynamic_projector_image_points(cv::Mat img, bool offset_from_marker)
{
	vector<cv::Point2f> chessImgPts;
	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();

	if (bPrintedPatternFound) {
		if (rotCamToProj.empty() || transCamToProj.empty()) {
			return false;
		}

		if (projector_calibrator.get_camera_matrix().empty() ||
			projector_calibrator.get_dist_coeffs().empty()) {
			return false;
		}

		drawChessboardCorners(img, camera_calibrator.get_board_size(), Mat(chessImgPts), bPrintedPatternFound);

		cv::Mat boardRot;
		cv::Mat boardTrans;
		camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);

		// Smooth the estimated board pose before reprojection to reduce visible jitter.
		const double poseAlpha = FLAGS_projector_smoothing_rate;
		if (!has_smoothed_dynamic_board_pose) {
			smoothed_dynamic_board_rot = boardRot.clone();
			smoothed_dynamic_board_trans = boardTrans.clone();
			has_smoothed_dynamic_board_pose = true;
		}
		else {
			smoothed_dynamic_board_rot =
				smoothed_dynamic_board_rot * (1.0 - poseAlpha) + boardRot * poseAlpha;
			smoothed_dynamic_board_trans =
				smoothed_dynamic_board_trans * (1.0 - poseAlpha) + boardTrans * poseAlpha;
		}

		const auto & camCandObjPts = camera_calibrator.get_candidate_object_points();
		Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
		Point3f axisY = camCandObjPts[camera_calibrator.get_board_size().width] - camCandObjPts[0];
		Point3f pos = camCandObjPts[0];
		if (offset_from_marker) {
			pos = camCandObjPts[0] - axisY * (camera_calibrator.get_board_size().width - 2) * static_cast<float>(FLAGS_projector_offset_y_scale);
		}

		vector<Point3f> auxObjectPoints;
		for (int i = 0; i < projector_calibrator.get_circle_pattern_size().height; i++) {
			for (int j = 0; j < projector_calibrator.get_circle_pattern_size().width; j++) {
				auxObjectPoints.push_back(pos + axisX * float((2 * j) + (i % 2)) + axisY * i);
			}
		}

		vector<Point2f> followingPatternImagePoints = get_projected(
			auxObjectPoints,
			smoothed_dynamic_board_rot,
			smoothed_dynamic_board_trans);

		const auto& prevCandidatePoints = projector_calibrator.get_candidate_image_points();
		if (!prevCandidatePoints.empty() && prevCandidatePoints.size() == followingPatternImagePoints.size()) {
			std::vector<cv::Point2f> smoothedPoints;
			smoothedPoints.reserve(followingPatternImagePoints.size());

			// Smooth motion to avoid sudden jumps when board pose detection jitters.
			const float alpha = static_cast<float>(std::clamp(FLAGS_projector_smoothing_rate, 0.0, 1.0));
			const float maxStepPx = 45.0f;

			for (size_t i = 0; i < followingPatternImagePoints.size(); ++i) {
				const cv::Point2f& prev = prevCandidatePoints[i];
				const cv::Point2f& next = followingPatternImagePoints[i];

				cv::Point2f blended = prev * (1.0f - alpha) + next * alpha;
				cv::Point2f delta = blended - prev;
				float deltaNorm = std::sqrt((delta.x * delta.x) + (delta.y * delta.y));
				if (deltaNorm > maxStepPx && deltaNorm > 1e-6f) {
					float scale = maxStepPx / deltaNorm;
					blended = prev + (delta * scale);
				}
				smoothedPoints.push_back(blended);
			}
			projector_calibrator.set_candidate_image_points(smoothedPoints);
		}
		else {
			projector_calibrator.set_candidate_image_points(followingPatternImagePoints);
		}
	}
	return bPrintedPatternFound;
}

bool Tinker::calibration::is_dynamic_projector_calibration_satisfied() const
{
	return dynamic_accepted_samples >= required_dynamic_projector_samples &&
		last_dynamic_stereo_rms <= dynamic_stereo_rms_threshold;
}

void Tinker::calibration::reset_dynamic_projection_priming()
{
	dynamic_projection_primed = false;
	has_smoothed_dynamic_board_pose = false;
	smoothed_dynamic_board_rot.release();
	smoothed_dynamic_board_trans.release();
	last_dynamic_stereo_rms = std::numeric_limits<double>::infinity();
}

bool Tinker::calibration::is_dynamic_projection_primed() const
{
	return dynamic_projection_primed;
}

void Tinker::calibration::set_dynamic_projection_primed(bool primed)
{
	dynamic_projection_primed = primed;
}

void Tinker::calibration::reset_dynamic_calibration_solution()
{
	dynamic_calibration_has_solution = false;
	dynamic_accepted_samples = 0;
}

void Tinker::calibration::mark_dynamic_calibration_solution()
{
	dynamic_calibration_has_solution = true;
	++dynamic_accepted_samples;
	std::cout << "Accepted dynamic projector sample "
		<< dynamic_accepted_samples << "/" << required_dynamic_projector_samples
		<< std::endl;
}

bool Tinker::calibration::has_dynamic_calibration_solution() const
{
	return dynamic_calibration_has_solution;
}

void Tinker::calibration::draw_camera_debug(Mat& image)
{
	if (!camera_calibrator.find_board(image)) {
		return;
	}

	const auto chessImgPts = camera_calibrator.get_detected_board_points();
	drawChessboardCorners(image, camera_calibrator.get_board_size(), Mat(chessImgPts), true);

	const cv::Mat cameraMatrix = camera_calibrator.get_camera_matrix();
	const cv::Mat distCoeffs = camera_calibrator.get_dist_coeffs();
	if (cameraMatrix.empty() || distCoeffs.empty()) {
		return;
	}

	cv::Mat boardRot;
	cv::Mat boardTrans;
	camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);

	const auto objectPoints = camera_calibrator.get_candidate_object_points();
	float axisLength = 1.0f;
	if (objectPoints.size() > 1) {
		axisLength = static_cast<float>(cv::norm(objectPoints[1] - objectPoints[0]) * 2.0);
	}

	cv::drawFrameAxes(image, cameraMatrix, distCoeffs, boardRot, boardTrans, axisLength, 2);
}

void Tinker::calibration::draw_projector_pattern(Mat& projectorImage)
{
	int radius = static_cast<int>(FLAGS_projected_circle_radius);
	projectorImage = cv::Mat::zeros(projectorImage.size(), projectorImage.type());
	vector<Point2f> points = projector_calibrator.get_candidate_image_points();
	for (int i = 0; i < points.size(); i++) {
		circle(projectorImage, points[i], radius, Scalar(255, 255, 255), -1, 8, 0);
	}
	
}

Mat Tinker::calibration::process_image_for_circle_detection(Mat img)
{
	Mat thresholdedImage;
	if (img.type() != CV_8UC1) {
		cvtColor(img, thresholdedImage, COLOR_BGR2GRAY);
	}
	else {
		img.copyTo(thresholdedImage);
	}
	cv::threshold(thresholdedImage, thresholdedImage, 210, 255, cv::THRESH_BINARY_INV);
	return thresholdedImage;
}

bool Tinker::calibration::calibrate_projector(Mat img)
{
	if (!accept_new_frame(img)) {
		return false;
	}

	Mat processedImage = process_image_for_circle_detection(img);

	if (add_projected(img, processedImage)) {
		
		cout << "calibrating projector inside calibrate_projector"  << endl;
		if (projector_calibrator.calibrate()) {
			cout << "projector calibration finished!" << endl;
			stereo_calibrate();

			return true;
		}
	}
	imshow("ImageThresholded", processedImage);
	return false;
}

void Tinker::calibration::stereo_calibrate()
{
	const auto & objectPoints = projector_calibrator.get_object_points();
	cout << "objectPoints size : " << objectPoints.size() << endl;

	auto n = objectPoints.size();
	if (projector_calibrator.imagePoints.size() != n ||
		projector_calibrator.frameMeasuredCircleImagePoints.size() != n ||
		projector_calibrator.camBoardRotations.size() != n ||
		projector_calibrator.camBoardTranslations.size() != n) {
		std::cerr << "Stereo input size mismatch: "
			<< "obj=" << n
			<< " projImg=" << projector_calibrator.imagePoints.size()
			<< " camImg=" << projector_calibrator.frameMeasuredCircleImagePoints.size()
			<< " refCamR=" << projector_calibrator.camBoardRotations.size()
			<< " refCamT=" << projector_calibrator.camBoardTranslations.size()
			<< std::endl;
		return;
	}
	const auto& auxImagePointsCamera = projector_calibrator.frameMeasuredCircleImagePoints;

	cv::Mat projectorMatrix = projector_calibrator.get_camera_matrix();
	cv::Mat projectorDistCoeffs = projector_calibrator.get_dist_coeffs();
	cv::Mat cameraMatrix = camera_calibrator.get_camera_matrix();
	cv::Mat cameraDistCoeffs = camera_calibrator.get_dist_coeffs();

	cv::Mat fundamentalMatrix, essentialMatrix;
	cv::Mat rotation3x3;

	if (cameraMatrix.empty() || cameraDistCoeffs.empty()) {
		std::cerr << "Camera intrinsics are empty!" << std::endl;
		return;
	}

	if (projectorMatrix.empty() || projectorDistCoeffs.empty()) {
		std::cerr << "Camera intrinsics are empty!" << std::endl;
		return;
	}

	const double stereoRms = cv::stereoCalibrate(objectPoints,
		auxImagePointsCamera,
		projector_calibrator.imagePoints,
		cameraMatrix, cameraDistCoeffs,
		projectorMatrix, projectorDistCoeffs,
		camera_calibrator.get_image_size(),
		rotation3x3, transCamToProj,
		essentialMatrix, fundamentalMatrix,
		CALIB_FIX_INTRINSIC);
	last_dynamic_stereo_rms = stereoRms;
	std::cout << "Dynamic stereo RMS error: " << stereoRms << std::endl;

	cv::Rodrigues(rotation3x3, rotCamToProj);
}


bool Tinker::calibration::add_projected(cv::Mat img, cv::Mat processedImg)
{
	vector<cv::Point2f> chessImgPts;

	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();
	if (bPrintedPatternFound) {
		Size board = camera_calibrator.get_board_size();
		
		drawChessboardCorners(img, board, Mat(chessImgPts), bPrintedPatternFound);
		drawChessboardCorners(processedImg, board, Mat(chessImgPts), bPrintedPatternFound);
		vector<cv::Point2f> circlesImgPts;
		bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, projector_calibrator.get_circle_pattern_size(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);

		if (bProjectedPatternFound) {
			drawChessboardCorners(img, projector_calibrator.get_circle_pattern_size(), Mat(circlesImgPts), bProjectedPatternFound);
			drawChessboardCorners(processedImg, projector_calibrator.get_circle_pattern_size(), Mat(circlesImgPts), bProjectedPatternFound);

			vector<cv::Point3f> circlesObjectPts;
			cv::Mat boardRot;
			cv::Mat boardTrans;

			camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
			camera_calibrator.back_project(boardRot, boardTrans, circlesImgPts, circlesObjectPts);

			// Store the measured image of circles as seen from the camera, to be used by the projector calibrator later
			projector_calibrator.frameMeasuredCircleImagePoints.push_back(circlesImgPts);

			camera_calibrator.imagePointsCamObj.push_back(chessImgPts);
			camera_calibrator.get_object_points().push_back(camera_calibrator.get_candidate_object_points());
			camera_calibrator.boardRotations.push_back(boardRot);
			camera_calibrator.boardTranslations.push_back(boardTrans);

			projector_calibrator.imagePoints.push_back(projector_calibrator.get_candidate_image_points());
			projector_calibrator.objectPoints.push_back(circlesObjectPts);

			// during the same frame where we computed camera pose, we also store that pose in the projector calibrator
			projector_calibrator.camBoardRotations.push_back(boardRot);
			projector_calibrator.camBoardTranslations.push_back(boardTrans);

			cout << "after add_projected : " << "imagePoints size : " << projector_calibrator.imagePoints.size() << endl;
			cout << "after add_projected : " << "objectPoints size : " << projector_calibrator.objectPoints.size() << endl;

			return true;
		}
		else {
			return false;
		}
	}
	return false;
}
