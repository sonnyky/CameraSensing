#include "include\calibration.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include "flags.hpp"
#include "projector_circle_detection.hpp"
#include "projector_projection_bounds.hpp"
#include <sstream>
#include "stereo_fit_validation.hpp"
#include "projection_region_layout.hpp"
#include "projector_distortion_validation.hpp"
#include "shared_pose_smoothing.hpp"

Tinker::calibration::calibration()
{
	camera_is_calibrated = false;
}

Tinker::calibration::~calibration()
{
}

void Tinker::calibration::setup_camera_calibration_parameters(Size boardSize_, Size imageSize_, float squareSize_, float aspectRatio_, int nFrames_, int delay_, int mode_, int cameraId_, string outputFileName_)
{
	camera_calibrator.setup_parameters(
		boardSize_,
		imageSize_,
		squareSize_,
		aspectRatio_,
		nFrames_,
		mode_,
		cameraId_,
		outputFileName_
	);
	minimum_sample_interval = std::chrono::milliseconds(delay_);
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
	projector_output_filename = _outputFileName;
	required_dynamic_projector_samples = static_cast<uint64_t>(_nFramesDynamicProjectorCalib);
}

void Tinker::calibration::set_projector_static_image_points()
{
	projector_calibrator.set_static_candidate_image_points();
}

bool Tinker::calibration::calibrate_camera(Mat image)
{
	if (!camera_calibrator.find_board(image)) {
		return false;
	}

	const auto boardPoints = camera_calibrator.get_detected_board_points();
	if (!should_accept_board_sample(boardPoints)) {
		return false;
	}

	const bool calibrationComplete = camera_calibrator.calibrate(image, boardPoints);
	commit_accepted_board_sample(boardPoints);
	return calibrationComplete;
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

bool Tinker::calibration::should_accept_board_sample(const vector<Point2f>& boardPoints, string* rejectionReason) const
{
	auto reject = [rejectionReason](const string& reason) {
		if (rejectionReason) *rejectionReason = reason;
		return false;
	};
	if (boardPoints.empty()) {
		return reject("chessboard corners missing");
	}

	if (!has_accepted_board_sample) {
		return true;
	}

	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		steady_clock::now() - last_accepted_sample_time).count();
	if (elapsed < minimum_sample_interval.count()) {
		return reject("waiting for --delay_between_frames interval; hold the board steady");
	}
	if (boardPoints.size() != last_accepted_board_points.size()) {
		return reject("chessboard corner count changed");
	}

	double sumSquaredDisplacement = 0.0;
	for (size_t i = 0; i < boardPoints.size(); ++i) {
		const cv::Point2f displacement = boardPoints[i] - last_accepted_board_points[i];
		sumSquaredDisplacement += displacement.dot(displacement);
	}

	const double rmsDisplacement = std::sqrt(sumSquaredDisplacement / boardPoints.size());
	if (rmsDisplacement < FLAGS_minimum_board_motion_px) {
		return reject("insufficient chessboard movement: " + std::to_string(rmsDisplacement) +
			" px < " + std::to_string(FLAGS_minimum_board_motion_px) + " px; move or tilt the board");
	}
	return true;
}

void Tinker::calibration::report_projector_capture_rejection(const string& reason)
{
	const auto now = steady_clock::now();
	// Throttle all failures, including changing numerical values, to avoid per-frame spam.
	if (last_projector_capture_rejection.empty() || now - last_projector_rejection_time >= std::chrono::seconds(2)) {
		cout << "Projector sample not captured: " << reason << endl;
		last_projector_rejection_time = now;
	}
	last_projector_capture_rejection = reason;
}

void Tinker::calibration::commit_accepted_board_sample(const vector<Point2f>& boardPoints)
{
	last_accepted_board_points = boardPoints;
	last_accepted_sample_time = steady_clock::now();
	has_accepted_board_sample = true;
}

void Tinker::calibration::report_projector_projection_issue(const string& reason, bool blanked)
{
	const auto now = steady_clock::now();
	if (last_projector_projection_issue.empty() || blanked != last_projector_projection_issue_blanked ||
		now - last_projector_projection_issue_time >= std::chrono::seconds(2)) {
		cout << (blanked ? "Projector projection blanked: " : "Projector projection bounds warning: ") << reason << endl;
		last_projector_projection_issue_time = now;
	}
	last_projector_projection_issue = reason;
	last_projector_projection_issue_blanked = blanked;
}

void Tinker::calibration::reset_sample_capture_gate()
{
	last_accepted_board_points.clear();
	has_accepted_board_sample = false;
	last_projector_capture_rejection.clear();
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
	cv::Mat rotation;
	cv::Rodrigues(rotObjToProj, rotation);
	const auto intrinsics = projector_calibrator.get_camera_matrix();
	double validatedRadius=0;
	const auto projectorSize=projector_calibrator.get_image_size();
	for (double x : {0.0,double(projectorSize.width-1)}) for (double y : {0.0,double(projectorSize.height-1)})
		validatedRadius=std::max(validatedRadius,std::hypot((x-intrinsics.at<double>(0,2))/intrinsics.at<double>(0,0),
			(y-intrinsics.at<double>(1,2))/intrinsics.at<double>(1,1)));
	validatedRadius *= 1.25;
	for (const auto& point : pts) {
		const double depth = rotation.at<double>(2,0)*point.x + rotation.at<double>(2,1)*point.y +
			rotation.at<double>(2,2)*point.z + transObjToProj.at<double>(2);
		if (!std::isfinite(depth) || depth <= 1e-6) return {};
		const double x=rotation.at<double>(0,0)*point.x+rotation.at<double>(0,1)*point.y+
			rotation.at<double>(0,2)*point.z+transObjToProj.at<double>(0);
		const double y=rotation.at<double>(1,0)*point.x+rotation.at<double>(1,1)*point.y+
			rotation.at<double>(1,2)*point.z+transObjToProj.at<double>(1);
		if (!std::isfinite(x) || !std::isfinite(y) || std::hypot(x/depth,y/depth)>validatedRadius) return {};
	}

	vector<Point2f> out;
	projectPoints(Mat(pts),
		rotObjToProj, transObjToProj,
		projector_calibrator.get_camera_matrix(),
		projector_calibrator.get_dist_coeffs(),
		out);
	return out;
}

bool Tinker::calibration::set_dynamic_projector_image_points(cv::Mat img, bool immediate_update)
{
	vector<cv::Point2f> chessImgPts;
	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();
	if (!bPrintedPatternFound) {
		has_smoothed_dynamic_board_pose = false;
		smoothed_dynamic_board_rot.release();
		smoothed_dynamic_board_trans.release();
		report_projector_projection_issue("chessboard/origin unavailable: " + camera_calibrator.get_orientation_status(), true);
		return false;
	}

	if (bPrintedPatternFound) {
		if (rotCamToProj.empty() || transCamToProj.empty()) {
			report_projector_projection_issue("camera-to-projector extrinsics are unavailable", true);
			return false;
		}

		if (projector_calibrator.get_camera_matrix().empty() ||
			projector_calibrator.get_dist_coeffs().empty()) {
			report_projector_projection_issue("projector intrinsics are unavailable", true);
			return false;
		}

		cv::Mat boardRot;
		cv::Mat boardTrans;
		camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
		string distortionReason;
		if (!valid_projector_distortion(projector_calibrator.get_camera_matrix(), projector_calibrator.get_dist_coeffs(),
			projector_calibrator.get_image_size(), distortionReason)) {
			report_projector_projection_issue("invalid projector distortion: " + distortionReason + "; recalibrate", true);
			return false;
		}

		// Calibration follows the measured pose immediately; smooth only tracking.
		const double poseAlpha = FLAGS_projector_smoothing_rate;
		if (immediate_update || !has_smoothed_dynamic_board_pose) {
			smoothed_dynamic_board_rot = boardRot.clone();
			smoothed_dynamic_board_trans = boardTrans.clone();
			has_smoothed_dynamic_board_pose = true;
		}
		else {
			smooth_shared_board_pose(boardRot, boardTrans, poseAlpha, smoothed_dynamic_board_rot, smoothed_dynamic_board_trans);
		}

		const auto & camCandObjPts = camera_calibrator.get_candidate_object_points();
		Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
		Point3f axisY = camCandObjPts[camera_calibrator.get_board_size().width] - camCandObjPts[0];
		Point3f pos = camCandObjPts[0];
		// Placement is identical in calibration and tracking. The phase only
		// controls smoothing, so entering tracking cannot jump onto the marker.
		{
			if (FLAGS_use_projection_region) {
				try {
					pos = centered_projection_grid_origin(camCandObjPts[0], axisX, axisY, projector_calibrator.get_circle_pattern_size(),
						FLAGS_projection_region_center_x_cm * 10, FLAGS_projection_region_top_cm * 10,
						FLAGS_projection_region_width_cm * 10, FLAGS_projection_region_height_cm * 10);
				} catch (const std::invalid_argument& error) {
					report_projector_projection_issue(error.what(), true);
					return false;
				}
			} else {
				pos = camCandObjPts[0] + axisY * (camera_calibrator.get_board_size().width - 2) * static_cast<float>(FLAGS_projector_offset_y_scale);
			}
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
		const auto desiredBounds = projector_projection_bounds(followingPatternImagePoints,
			projector_calibrator.get_image_size(), static_cast<double>(FLAGS_projected_circle_radius));
		if (followingPatternImagePoints.empty() || desiredBounds.finiteCenters != followingPatternImagePoints.size()) {
			report_projector_projection_issue("computed grid is behind the projector, beyond the validated distortion domain, or non-finite; check board pose and calibration", true);
			return false;
		}
		// Calibration requires a complete detectable grid. Tracking accumulates
		// no samples and may render the visible portion of a valid projection.
		if (immediate_update && desiredBounds.fullyVisibleCircles != followingPatternImagePoints.size()) {
			report_projector_projection_issue("complete grid does not fit the projector image (full circles=" +
				std::to_string(desiredBounds.fullyVisibleCircles) + "/" + std::to_string(followingPatternImagePoints.size()) +
				"); move the board into coverage; coordinates are not clamped", true);
			return false;
		}
		// Project every circle from one shared pose; no independent point
		// blending or step cap can deform the grid during tracking.
		projector_calibrator.set_candidate_image_points(followingPatternImagePoints);
		const auto& displayedPoints = projector_calibrator.get_candidate_image_points();
		const auto displayedBounds = projector_projection_bounds(displayedPoints, projector_calibrator.get_image_size(),
			static_cast<double>(FLAGS_projected_circle_radius));
		if (desiredBounds.fullyVisibleCircles != followingPatternImagePoints.size() ||
			displayedBounds.fullyVisibleCircles != displayedPoints.size()) {
			std::ostringstream message;
			message << "target full circles=" << desiredBounds.fullyVisibleCircles << "/" << followingPatternImagePoints.size()
				<< ", target center range X=[" << desiredBounds.minX << "," << desiredBounds.maxX
				<< "], Y=[" << desiredBounds.minY << "," << desiredBounds.maxY
				<< "]; " << (immediate_update ? "direct update full circles=" : "smoothed update full circles=")
				<< displayedBounds.fullyVisibleCircles << "/" << displayedPoints.size()
				<< ", intersecting circle bounds=" << displayedBounds.intersectingCircles
				<< "; projector image=" << projector_calibrator.get_image_size().width << "x" << projector_calibrator.get_image_size().height
				<< ". Coordinates are not clamped; check board pose, placement and calibration.";
			report_projector_projection_issue(message.str(), false);
		}
		else {
			last_projector_projection_issue.clear();
		}
	}
	return bPrintedPatternFound;
}

bool Tinker::calibration::is_dynamic_projector_calibration_satisfied() const
{
	return dynamic_accepted_samples >= required_dynamic_projector_samples &&
		last_dynamic_stereo_rms <= FLAGS_max_dynamic_stereo_rms;
}

void Tinker::calibration::reset_dynamic_projection_priming()
{
	dynamic_projection_primed = false;
	last_projector_projection_issue.clear();
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
		cv::putText(image, camera_calibrator.get_orientation_status(), Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
		return;
	}

	const auto chessImgPts = camera_calibrator.get_detected_board_points();
	drawChessboardCorners(image, camera_calibrator.get_board_size(), Mat(chessImgPts), true);
	cv::circle(image, chessImgPts.front(), 7, Scalar(0, 255, 255), 2);
	cv::putText(image, "origin (0,0)", chessImgPts.front() + Point2f(5, -8), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
	cv::putText(image, camera_calibrator.get_orientation_status(), Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);

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
	if (FLAGS_use_projection_region) {
		const float left = static_cast<float>((FLAGS_projection_region_center_x_cm - FLAGS_projection_region_width_cm / 2) * 10);
		const float right = left + static_cast<float>(FLAGS_projection_region_width_cm * 10);
		const float top = static_cast<float>(FLAGS_projection_region_top_cm * 10);
		const float bottom = top + static_cast<float>(FLAGS_projection_region_height_cm * 10);
		vector<Point3f> region = {{left, top, 0}, {right, top, 0}, {right, bottom, 0}, {left, bottom, 0}};
		vector<Point2f> regionPixels;
		cv::projectPoints(region, boardRot, boardTrans, cameraMatrix, distCoeffs, regionPixels);
		for (size_t index = 0; index < regionPixels.size(); ++index) {
			const auto& a = regionPixels[index]; const auto& b = regionPixels[(index + 1) % regionPixels.size()];
			if (std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(b.x) && std::isfinite(b.y) &&
				cv::norm(a) < 1e6 && cv::norm(b) < 1e6) cv::line(image, a, b, Scalar(255, 255, 0), 2);
		}
	}
}

void Tinker::calibration::draw_projector_pattern(Mat& projectorImage)
{
	int radius = static_cast<int>(FLAGS_projected_circle_radius);
	projectorImage = cv::Mat::zeros(projectorImage.size(), projectorImage.type());
	vector<Point2f> points = projector_calibrator.get_candidate_image_points();
	for (int i = 0; i < points.size(); i++) {
		if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
			points[i].x + radius < 0 || points[i].y + radius < 0 ||
			points[i].x - radius >= projectorImage.cols || points[i].y - radius >= projectorImage.rows) {
			continue; // Do not convert non-finite or wholly off-image centers to integer pixels.
		}
		circle(projectorImage, points[i], radius, Scalar(255, 255, 255), -1, 8, 0);
	}
	
}

Mat Tinker::calibration::process_image_for_circle_detection(Mat img)
{
	return threshold_projector_circles(img);
}

void Tinker::calibration::record_displayed_projector_pattern(bool hasPattern)
{
	displayed_projector_pattern.record(projector_calibrator.get_candidate_image_points(), hasPattern);
}

bool Tinker::calibration::calibrate_projector(Mat img)
{
	if (displayed_projector_pattern.points().empty()) {
		report_projector_capture_rejection("no recorded pattern on screen; skipping blank/stale-pattern capture");
		return false;
	}
	const bool dynamicSample = dynamic_projection_primed;
	Mat processedImage = process_image_for_circle_detection(img);
	if (!camera_calibrator.find_board(img)) {
		report_projector_capture_rejection("chessboard/origin unavailable: " + camera_calibrator.get_orientation_status());
		imshow("ImageThresholded", processedImage);
		return false;
	}

	const auto boardPoints = camera_calibrator.get_detected_board_points();
	string rejectionReason;
	if (!should_accept_board_sample(boardPoints, &rejectionReason)) {
		report_projector_capture_rejection(rejectionReason);
		imshow("ImageThresholded", processedImage);
		return false;
	}

	if (add_projected(img, processedImage)) {
		if (!dynamicSample) commit_accepted_board_sample(boardPoints);
		last_projector_capture_rejection.clear();
		
		cout << "calibrating projector inside calibrate_projector"  << endl;
		if (projector_calibrator.calibrate(camera_calibrator.get_image_size())) {
			cout << "projector calibration finished!" << endl;
			if (stereo_calibrate()) {
				if (dynamicSample) commit_accepted_board_sample(boardPoints);
				return true;
			}
			if (dynamicSample) {
				projector_calibrator.imagePoints.pop_back();
				projector_calibrator.objectPoints.pop_back();
				projector_calibrator.frameMeasuredCircleImagePoints.pop_back();
				projector_calibrator.frameMeasuredBoardImagePoints.pop_back();
				projector_calibrator.camBoardRotations.pop_back();
				projector_calibrator.camBoardTranslations.pop_back();
				camera_calibrator.imagePointsCamObj.pop_back();
				camera_calibrator.get_object_points().pop_back();
				camera_calibrator.boardRotations.pop_back();
				camera_calibrator.boardTranslations.pop_back();
				cout << "Rejected sample discarded; previous stereo transform and accepted views retained." << endl;
			}
		}
	}
	imshow("ImageThresholded", processedImage);
	return false;
}

bool Tinker::calibration::stereo_calibrate()
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
		return false;
	}
	const auto& auxImagePointsCamera = projector_calibrator.frameMeasuredCircleImagePoints;

	cv::Mat projectorMatrix = projector_calibrator.get_camera_matrix();
	cv::Mat projectorDistCoeffs = projector_calibrator.get_dist_coeffs();
	cv::Mat cameraMatrix = camera_calibrator.get_camera_matrix();
	cv::Mat cameraDistCoeffs = camera_calibrator.get_dist_coeffs();

	cv::Mat fundamentalMatrix, essentialMatrix;
	cv::Mat rotation3x3;
	cv::Mat candidateTranslation;

	if (cameraMatrix.empty() || cameraDistCoeffs.empty()) {
		std::cerr << "Camera intrinsics are empty!" << std::endl;
		return false;
	}

	if (projectorMatrix.empty() || projectorDistCoeffs.empty()) {
		std::cerr << "Camera intrinsics are empty!" << std::endl;
		return false;
	}

	double stereoRms = std::numeric_limits<double>::infinity();
	try {
		stereoRms = cv::stereoCalibrate(objectPoints,
		auxImagePointsCamera,
		projector_calibrator.imagePoints,
		cameraMatrix, cameraDistCoeffs,
		projectorMatrix, projectorDistCoeffs,
		camera_calibrator.get_image_size(),
		rotation3x3, candidateTranslation,
		essentialMatrix, fundamentalMatrix,
		CALIB_FIX_INTRINSIC);
	}
	catch (const cv::Exception& error) {
		cout << "Stereo fit rejected: OpenCV could not solve candidate views; working transform unchanged. "
			<< error.what() << endl;
		return false;
	}
	std::cout << "Candidate stereo RMS error: " << stereoRms << std::endl;
	if (!commit_valid_stereo_fit(stereoRms, FLAGS_max_dynamic_stereo_rms, rotation3x3,
		candidateTranslation, rotCamToProj, transCamToProj, last_dynamic_stereo_rms)) {
		cout << "Stereo fit rejected: invalid transform or RMS above " << FLAGS_max_dynamic_stereo_rms
			<< " px; working transform unchanged." << endl;
		return false;
	}
	cout << "Stereo fit accepted: RMS=" << stereoRms << " px" << endl;
	return true;
}

void Tinker::calibration::save_stereo_calibration() const
{
	if (projector_output_filename.empty() || rotCamToProj.empty() || transCamToProj.empty() ||
		!std::isfinite(last_dynamic_stereo_rms)) {
		throw std::runtime_error("Cannot save camera-to-projector extrinsics before stereo calibration succeeds.");
	}

	cv::Mat rotationMatrix;
	cv::Rodrigues(rotCamToProj, rotationMatrix);

	cv::FileStorage fs(projector_output_filename, cv::FileStorage::APPEND);
	if (!fs.isOpened()) {
		throw std::runtime_error("Unable to append stereo calibration to: " + projector_output_filename);
	}

	fs << "Rotation_Vector" << rotCamToProj;
	fs << "Rotation_Matrix" << rotationMatrix;
	fs << "Translation_Vector" << transCamToProj;
	fs << "stereo_rms_error" << last_dynamic_stereo_rms;
}


bool Tinker::calibration::add_projected(cv::Mat img, cv::Mat& processedImg)
{
	vector<cv::Point2f> chessImgPts;

	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();
	if (bPrintedPatternFound) {
		Size board = camera_calibrator.get_board_size();
		
		vector<cv::Point2f> circlesImgPts;
		vector<cv::KeyPoint> blobs;
		bool bProjectedPatternFound = detect_projector_circles(processedImg,
			projector_calibrator.get_circle_pattern_size(), circlesImgPts, blobs);
		// Only annotate after detection; never feed debug graphics into the detector.
		Mat debugImage;
		cv::drawKeypoints(processedImg, blobs, debugImage, Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		processedImg = debugImage;
		cv::putText(processedImg, "Blob candidates: " + std::to_string(blobs.size()) + " (grid needs 20)",
			Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
		drawChessboardCorners(img, board, Mat(chessImgPts), bPrintedPatternFound);
		drawChessboardCorners(processedImg, board, Mat(chessImgPts), bPrintedPatternFound);

		if (bProjectedPatternFound) {
			drawChessboardCorners(img, projector_calibrator.get_circle_pattern_size(), Mat(circlesImgPts), bProjectedPatternFound);
			drawChessboardCorners(processedImg, projector_calibrator.get_circle_pattern_size(), Mat(circlesImgPts), bProjectedPatternFound);

			vector<cv::Point3f> circlesObjectPts;
			cv::Mat boardRot;
			cv::Mat boardTrans;

			camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
			if (!camera_calibrator.back_project(boardRot, boardTrans, circlesImgPts, circlesObjectPts)) {
				report_projector_capture_rejection("circle back-projection failed: invalid or grazing board-plane rays");
				return false;
			}

			// Store the measured image of circles as seen from the camera, to be used by the projector calibrator later
			projector_calibrator.frameMeasuredCircleImagePoints.push_back(circlesImgPts);
			projector_calibrator.frameMeasuredBoardImagePoints.push_back(chessImgPts);

			camera_calibrator.imagePointsCamObj.push_back(chessImgPts);
			camera_calibrator.get_object_points().push_back(camera_calibrator.get_candidate_object_points());
			camera_calibrator.boardRotations.push_back(boardRot);
			camera_calibrator.boardTranslations.push_back(boardTrans);

			projector_calibrator.imagePoints.push_back(displayed_projector_pattern.points());
			projector_calibrator.objectPoints.push_back(circlesObjectPts);

			// during the same frame where we computed camera pose, we also store that pose in the projector calibrator
			projector_calibrator.camBoardRotations.push_back(boardRot);
			projector_calibrator.camBoardTranslations.push_back(boardTrans);

			cout << "after add_projected : " << "imagePoints size : " << projector_calibrator.imagePoints.size() << endl;
			cout << "after add_projected : " << "objectPoints size : " << projector_calibrator.objectPoints.size() << endl;

			return true;
		}
		else {
			report_projector_capture_rejection("complete 4x5 circle grid not detected; blob candidates=" + std::to_string(blobs.size()) +
				" (need 20 grid points, candidates may include clutter). Check threshold/blob settings and ImageThresholded; all circles must lie on the chessboard plane");
			return false;
		}
	}
	report_projector_capture_rejection("chessboard detection failed during circle capture");
	return false;
}
