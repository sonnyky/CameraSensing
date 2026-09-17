#include "camera_calibration.hpp"
#include "calibration_view_selection.hpp"
#include "flags.hpp"
#include <algorithm>
#include <opencv2/objdetect/aruco_detector.hpp>

Tinker::camera_calibration::camera_calibration()
{

}

Tinker::camera_calibration::~camera_calibration()
{
	calibrationStatus = CAPTURING;
}

void Tinker::camera_calibration::setup_parameters(cv::Size boardSize_, cv::Size imageSize_, float squareSize_, float aspectRatio_, int nFrames_, int mode_, int cameraId_, std::string outputFileName_)
{
	boardSize = boardSize_;
	orientation_tracker.reset();
	imageSize = imageSize_;
	patternLengthInRealUnits = squareSize_;
	aspectRatio = aspectRatio_;
	nframes = nFrames_;
	calibrationStatus = mode_;
	cameraId = cameraId_;
	outputFilename = outputFileName_;
	load_camera_matrix(outputFileName_);
}

bool Tinker::camera_calibration::calibrate(Mat image_, const vector<Point2f>& detectedPoints)
{
	Mat viewGray;

	const Pattern calibPattern = CHESSBOARD;
	if (!detectedPoints.empty()) {
		pointbuf = detectedPoints;
		found = true;
	}
	else {
		found = findChessboardCorners(image_, boardSize, pointbuf,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		cvtColor(image_, viewGray, COLOR_BGR2GRAY);
		if (calibPattern == CHESSBOARD && found) {
			cornerSubPix(viewGray, pointbuf, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}
	}

	if (found) {
		imagePoints.push_back(pointbuf);
	}

	const size_t candidateTarget = static_cast<size_t>(nframes) +
		static_cast<size_t>(FLAGS_calibration_candidate_margin);
	if (imagePoints.size() >= candidateTarget)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> perViewRms;
		double totalAvgErr = 0;

		const bool preliminarySuccess = runCalibration(imagePoints, imageSize, boardSize, calibPattern, patternLengthInRealUnits,
			aspectRatio, flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, perViewRms, totalAvgErr);
		printf("%s. avg reprojection error for this batch = %.2f\n",
			preliminarySuccess ? "Preliminary camera calibration succeeded" : "Preliminary camera calibration failed",
			totalAvgErr);
		if (!preliminarySuccess) {
			return false;
		}

		const auto selection = select_calibration_views(
			perViewRms, rvecs, tvecs, imagePoints, imageSize,
			static_cast<size_t>(nframes), FLAGS_max_camera_per_view_rms,
			FLAGS_minimum_calibration_position_span,
			FLAGS_minimum_calibration_distance_ratio,
			FLAGS_minimum_calibration_orientation_span_deg);
		std::cout << "Camera selection: eligible threshold=" << selection.robustRmsThreshold
			<< ", position span=" << selection.positionSpan
			<< ", distance ratio=" << selection.distanceRatio
			<< ", orientation span=" << selection.orientationSpanDegrees << " deg" << std::endl;
		if (!selection.hasEnoughQualityViews || !selection.hasRequiredCoverage) {
			std::cout << "Camera calibration needs more high-quality, diverse views." << std::endl;
			return false;
		}

		std::vector<std::vector<cv::Point2f>> selectedImagePoints;
		selectedImagePoints.reserve(selection.indices.size());
		for (size_t index : selection.indices) {
			selectedImagePoints.push_back(imagePoints[index]);
		}

		rvecs.clear();
		tvecs.clear();
		perViewRms.clear();
		totalAvgErr = 0.0;
		const bool finalSuccess = runCalibration(selectedImagePoints, imageSize, boardSize, calibPattern,
			patternLengthInRealUnits, aspectRatio, flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, perViewRms, totalAvgErr);
		printf("%s. avg reprojection error = %.2f\n",
			finalSuccess ? "Final camera calibration succeeded" : "Final camera calibration failed",
			totalAvgErr);
		if (!finalSuccess || totalAvgErr > FLAGS_max_camera_rms) {
			std::cout << "Camera aggregate RMS is above the completion threshold." << std::endl;
			return false;
		}

		imagePoints.swap(selectedImagePoints);
		saveCameraParams(outputFilename, imageSize,
			boardSize, patternLengthInRealUnits, aspectRatio,
			flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, perViewRms, imagePoints, totalAvgErr);
		load_camera_matrix(outputFilename);
		return true;
	}
	else {
		cout << "more camera candidates needed. we currently have: " << imagePoints.size()
			<< "/" << candidateTarget << " views." << endl;
	}

	return false;

}

void Tinker::camera_calibration::set_to_calibration_mode()
{
	imagePoints.clear();
	calibrationStatus = CAPTURING;
}

void Tinker::camera_calibration::undistort_image(Mat image)
{
}

void Tinker::camera_calibration::setup_candidate_object_points()
{
	candidateObjectPts.clear();
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			candidateObjectPts.push_back(cv::Point3f(float(j * patternLengthInRealUnits), float(i * patternLengthInRealUnits), 0));
		}
	}
}

void Tinker::camera_calibration::load(string camera_config)
{
}

bool Tinker::camera_calibration::find_board(Mat img)
{
	detected_board_points.clear();
	Mat gray;
	if (img.channels() == 3) {
		cvtColor(img, gray, COLOR_BGR2GRAY);
	}
	else {
		gray = img;
	}

	vector<BoardOrientationMarker> markers;
	Mat chessGray = gray;
	if (FLAGS_require_board_orientation) {
		cv::aruco::DetectorParameters parameters;
		parameters.minMarkerPerimeterRate = 0.01;
		parameters.adaptiveThreshWinSizeMax = 53;
		parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
		cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(FLAGS_board_aruco_dictionary), parameters);
		vector<int> ids;
		vector<vector<Point2f>> markerCorners;
		detector.detectMarkers(gray, markerCorners, ids);
		const auto isAnchor = [](int id) { return id == FLAGS_board_origin_aruco_id || id == FLAGS_board_origin_secondary_aruco_id; };
		if (std::none_of(ids.begin(), ids.end(), isAnchor)) {
			// Retry small codes at a larger sampling scale; this cannot restore
			// detail lost to blur, but can recover thresholding/contour failures.
			Mat enlarged;
			resize(gray, enlarged, Size(), 2, 2, INTER_CUBIC);
			detector.detectMarkers(enlarged, markerCorners, ids);
			for (auto& polygon : markerCorners) for (auto& point : polygon) point *= 0.5f;
		}
		chessGray = gray.clone();
		for (size_t index = 0; index < ids.size(); ++index) {
			if (ids[index] != FLAGS_board_origin_aruco_id && ids[index] != FLAGS_board_origin_secondary_aruco_id) continue;
			Point2f center;
			vector<Point> polygon;
			for (const auto& point : markerCorners[index]) { center += point * 0.25f; polygon.emplace_back(cvRound(point.x), cvRound(point.y)); }
			markers.push_back({ids[index], center});
			// These markers are on white squares. Remove their black interiors
			// from the private chessboard image so they do not bias corner refinement.
			fillConvexPoly(chessGray, polygon, Scalar(255));
		}
	}
	bool foundChessCorners = findChessboardCorners(chessGray, boardSize, detected_board_points,
		CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

	if (foundChessCorners) {
		cornerSubPix(chessGray, detected_board_points, Size(11, 11),
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		if (FLAGS_require_board_orientation) {
			foundChessCorners = orientation_tracker.orient(detected_board_points, boardSize, markers, orientation_status);
		}
		else orientation_status = "origin anchoring disabled";
	}
	else {
		orientation_tracker.reset();
		orientation_status = "complete chessboard missing; readable anchor markers=" + std::to_string(markers.size()) +
			"; keep all 9x6 inner corners visible";
	}
	if (!foundChessCorners && FLAGS_require_board_orientation && markers.empty() && !detected_board_points.empty())
		orientation_status += "; no readable anchor IDs (check marker size, blur and dictionary)";
	if (!foundChessCorners) detected_board_points.clear();

	return foundChessCorners;
}



void Tinker::camera_calibration::compute_candidate_board_pose(const vector<cv::Point2f>& imgPts, cv::Mat & boardRot, cv::Mat & boardTrans)
{
	/*cout << "candidateObjectPts  : " << candidateObjectPts.size() << endl;
	cout << "imgPts  : " << imgPts.size() << endl;*/

	cv::solvePnP(candidateObjectPts, imgPts,
		cameraMatrix,
		distCoeffs,
		boardRot, boardTrans);
}

bool Tinker::camera_calibration::back_project(const Mat & boardRot64, const Mat & boardTrans64, const vector<Point2f>& imgPt, vector<Point3f>& worldPt)
{
	if (imgPt.empty() || cameraMatrix.empty() || !checkRange(boardRot64) || !checkRange(boardTrans64)) return false;
	vector<Point2d> observed(imgPt.begin(), imgPt.end()), normalized;
	// Remove lens distortion BEFORE intersecting camera rays with the board plane.
	undistortPoints(observed, normalized, cameraMatrix, distCoeffs);
	Mat rotation, translation, rotationVector;
	boardRot64.convertTo(rotationVector, CV_64F);
	boardTrans64.convertTo(translation, CV_64F);
	Rodrigues(rotationVector, rotation);
	Mat inverseRotation = rotation.t();
	Mat cameraOriginInBoard = inverseRotation * translation;
	vector<Point3f> reconstructed;
	for (const auto& point : normalized) {
		Mat ray = (Mat_<double>(3, 1) << point.x, point.y, 1.0);
		Mat boardRay = inverseRotation * ray;
		const double denominator = boardRay.at<double>(2);
		if (!std::isfinite(denominator) || std::abs(denominator) < 1e-10) return false;
		const double scale = cameraOriginInBoard.at<double>(2) / denominator;
		Mat boardPoint = scale * boardRay - cameraOriginInBoard;
		if (!std::isfinite(scale) || scale <= 0 || !checkRange(boardPoint)) return false;
		reconstructed.emplace_back(static_cast<float>(boardPoint.at<double>(0)),
			static_cast<float>(boardPoint.at<double>(1)), 0.0f);
	}
	worldPt.insert(worldPt.end(), reconstructed.begin(), reconstructed.end());
	return true;
}

void Tinker::camera_calibration::load_camera_matrix(string fileName)
{

	// check for previous calibration files with the same file name
	struct stat buffer;
	bool found = stat(fileName.c_str(), &buffer) == 0;
	cout << "Camera calibration file exists : " << found << endl;
	if (found) {
		camera_is_calibrated = true;
		FileStorage fs(fileName, FileStorage::READ);
		fs["camera_matrix"] >> cameraMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
	}
}

double Tinker::camera_calibration::computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints, const vector<vector<Point2f>>& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat & cameraMatrix, const Mat & distCoeffs, vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

void Tinker::camera_calibration::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float(j*squareSize),
					float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
					float(i*squareSize), 0));
		break;

	default:
		CV_Error(Error::StsBadArg, "Unknown pattern type\n");
	}
}

bool Tinker::camera_calibration::runCalibration(vector<vector<Point2f>> imagePoints, 
	Size imageSize, Size boardSize, Pattern patternType, float squareSize, 
	float aspectRatio, int flags, Mat & cameraMatrix, Mat & distCoeffs, 
	vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& perViewRms, double & totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool matricesValid = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, perViewRms);

	return matricesValid;
}

void Tinker::camera_calibration::saveCameraParams(const string & filename, Size imageSize, Size boardSize, float squareSize, float aspectRatio, int flags, const Mat & cameraMatrix, const Mat & distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f>>& imagePoints, double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)
	{
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}
