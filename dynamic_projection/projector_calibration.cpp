#include "include\projector_calibration.hpp"
#include "calibration_view_selection.hpp"
#include "flags.hpp"
#include <algorithm>
#include <type_traits>
#include <stdexcept>
#include "projector_distortion_validation.hpp"

Tinker::projector_calibration::projector_calibration()
{
}

Tinker::projector_calibration::~projector_calibration()
{
}

void Tinker::projector_calibration::load(string projector_config)
{
    load_calibration_parameters(projector_config);
}

void Tinker::projector_calibration::set_static_candidate_image_points()
{
	candidate_image_points.clear();

	const int screenWidth = imageSize.width;
	const int screenHeight = imageSize.height;
	const float spacing = squareSize;

	// Calculate total width and height of the asymmetric circle grid
	float patternWidth = ((2 * circlePatternSize.width) - 1) * spacing;
	float patternHeight = (circlePatternSize.height - 1) * spacing;

	// Center the pattern in the middle of the projector screen
	patternPosition.x = static_cast<float>(screenWidth * FLAGS_static_projector_center_x - patternWidth / 2.0);
	patternPosition.y = static_cast<float>(screenHeight * FLAGS_static_projector_center_y - patternHeight / 2.0);
	const double radius = static_cast<double>(FLAGS_projected_circle_radius);
	if (patternPosition.x - radius < 0 || patternPosition.y - radius < 0 ||
		patternPosition.x + patternWidth + radius >= screenWidth ||
		patternPosition.y + patternHeight + radius >= screenHeight) {
		throw std::invalid_argument("Static projector grid is clipped: reduce spacing/radius or move its center inward.");
	}
	cout << "Static projector grid footprint: " << patternWidth + 2 * radius << " x "
		<< patternHeight + 2 * radius << " pixels; center="
		<< screenWidth * FLAGS_static_projector_center_x << ","
		<< screenHeight * FLAGS_static_projector_center_y << endl;
	const double centerSpan = std::hypot(patternWidth / screenWidth, patternHeight / screenHeight);
	if (centerSpan < FLAGS_minimum_projector_pattern_span) {
		throw std::invalid_argument("Static grid is too small for --minimum_projector_pattern_span; increase spacing before collecting calibration frames.");
	}

	// Generate circle points
	for (int i = 0; i < circlePatternSize.height; i++) {
		for (int j = 0; j < circlePatternSize.width; j++) {
			Point2f p;
			p.x = patternPosition.x + ((2 * j) + (i % 2)) * spacing;
			p.y = patternPosition.y + i * spacing;
			candidate_image_points.push_back(p);
		}
	}
}

void Tinker::projector_calibration::set_candidate_image_points(vector<cv::Point2f> pts)
{
	candidate_image_points = pts;
}

void Tinker::projector_calibration::setPatternPosition(float px, float py)
{
	patternPosition = Point2f(px, py);
}

/*
when image points and projected object points pairs are obtained on the camera, we use PnP to get board rotations and translations
*/
bool Tinker::projector_calibration::calibrate(Size cameraImageSize)
{
	const size_t candidateTarget = static_cast<size_t>(nFramesBeforeDynamcProjectorCalib) +
		static_cast<size_t>(FLAGS_calibration_candidate_margin);
	cout << "current projector candidate count: " << imagePoints.size()
		<< "/" << candidateTarget << endl;

	if (projector_is_calibrated) {
		return true;
	}

	if (imagePoints.size() >= candidateTarget) {
		cout << "got enough points for projector intrinsics calibration." << endl;

		// imagePointsProjObj and objectPoints has to have the same length
		if (imagePoints.size() != objectPoints.size() || imagePoints.size() != frameMeasuredBoardImagePoints.size() ||
			imagePoints.size() != frameMeasuredCircleImagePoints.size() || imagePoints.size() != camBoardRotations.size() ||
			imagePoints.size() != camBoardTranslations.size()) {
			cout << "Mismatched sizes. imagePointsProjObj : " << imagePoints.size()
				<< "and objectPoints : "<< objectPoints.size() << endl;
			return false;
		}

		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0.0;
		Mat fittedMatrix, fittedDistortion;

		if (!runCalibration(imagePoints, objectPoints, imageSize, 1, 0,
			fittedMatrix, fittedDistortion, rvecs, tvecs, reprojErrs, totalAvgErr)) {
			cout << "Projector preliminary fit failed intrinsic/distortion validation; working model unchanged." << endl;
			return false;
		}

		const auto selection = select_calibration_views(
			reprojErrs, camBoardRotations, camBoardTranslations,
			frameMeasuredBoardImagePoints, cameraImageSize,
			static_cast<size_t>(nFramesBeforeDynamcProjectorCalib),
			FLAGS_max_projector_per_view_rms,
			FLAGS_minimum_projector_board_position_span,
			FLAGS_minimum_calibration_distance_ratio,
			FLAGS_minimum_calibration_orientation_span_deg);
		for (size_t i = 0; i < reprojErrs.size(); ++i) {
			const bool eligible = std::isfinite(reprojErrs[i]) && reprojErrs[i] <= selection.robustRmsThreshold;
			const bool retained = std::find(selection.indices.begin(), selection.indices.end(), i) != selection.indices.end();
			cout << "Projector candidate " << i + 1 << ": preliminary per-view RMS=" << reprojErrs[i]
				<< " px; " << (!eligible ? "RMS-rejected" : !selection.hasEnoughQualityViews ? "eligible (diversity selection not evaluated)" :
					retained ? "selected for refit" : "eligible, not selected") << endl;
		}
		cout << "Projector selection: quality views=" << selection.eligibleViewCount << "/" << nFramesBeforeDynamcProjectorCalib
			<< ", eligible RMS threshold=" << selection.robustRmsThreshold << endl;
		if (!selection.hasEnoughQualityViews) {
			cout << "Projector board coverage: not evaluated (too few RMS-eligible views)." << endl;
			cout << "Projector RMS rejection: not enough views below the per-view threshold; improve focus, exposure and board flatness." << endl;
			return false;
		}
		cout << "Projector board coverage: position span=" << selection.positionSpan
			<< ", distance ratio=" << selection.distanceRatio
			<< ", orientation span=" << selection.orientationSpanDegrees << " deg" << endl;
		bool coveragePassed = true;
		auto requireCoverage = [&coveragePassed](const char* name, double actual, double required, const char* advice) {
			if (actual < required) {
				cout << "Projector coverage rejection: " << name << "=" << actual << " < " << required << ". " << advice << endl;
				coveragePassed = false;
			}
		};
		requireCoverage("board position span", selection.positionSpan, FLAGS_minimum_projector_board_position_span, "Translate the board while keeping both patterns visible.");
		requireCoverage("board distance ratio", selection.distanceRatio, FLAGS_minimum_calibration_distance_ratio, "Move the board nearer/farther.");
		requireCoverage("board tilt span (deg)", selection.orientationSpanDegrees, FLAGS_minimum_calibration_orientation_span_deg, "Tilt the board in different directions.");
		Point2f minimumPoint(static_cast<float>(imageSize.width), static_cast<float>(imageSize.height));
		Point2f maximumPoint(0, 0);
		for (size_t index : selection.indices) {
			for (const auto& point : imagePoints[index]) {
				minimumPoint.x = std::min(minimumPoint.x, point.x);
				minimumPoint.y = std::min(minimumPoint.y, point.y);
				maximumPoint.x = std::max(maximumPoint.x, point.x);
				maximumPoint.y = std::max(maximumPoint.y, point.y);
			}
		}
		const double projectorSpan = std::hypot((maximumPoint.x - minimumPoint.x) / imageSize.width,
			(maximumPoint.y - minimumPoint.y) / imageSize.height);
		cout << "Projector-image circle-center footprint span=" << projectorSpan << endl;
		requireCoverage("projector pattern span", projectorSpan, FLAGS_minimum_projector_pattern_span, "Increase static spacing; extra board movement cannot enlarge a fixed projector grid.");
		if (!coveragePassed) {
			return false;
		}

		auto gatherSelected = [&selection](const auto& views) {
			using ViewType = typename std::decay_t<decltype(views)>::value_type;
			std::vector<ViewType> selected;
			selected.reserve(selection.indices.size());
			for (size_t index : selection.indices) {
				selected.push_back(views[index]);
			}
			return selected;
		};
		auto selectedImagePoints = gatherSelected(imagePoints);
		auto selectedObjectPoints = gatherSelected(objectPoints);
		auto selectedMeasuredPoints = gatherSelected(frameMeasuredCircleImagePoints);
		auto selectedMeasuredBoardPoints = gatherSelected(frameMeasuredBoardImagePoints);
		auto selectedCameraRotations = gatherSelected(camBoardRotations);
		auto selectedCameraTranslations = gatherSelected(camBoardTranslations);

		rvecs.clear();
		tvecs.clear();
		reprojErrs.clear();
		totalAvgErr = 0.0;

		if (runCalibration(selectedImagePoints, selectedObjectPoints, imageSize, 1, 0,
			fittedMatrix, fittedDistortion, rvecs, tvecs, reprojErrs, totalAvgErr)) {
			last_avg_reprojection_error = totalAvgErr;
			last_per_view_reprojection_errors = reprojErrs;
			printf("Projector avg reprojection error after filtering = %.2f\n", totalAvgErr);
			for (size_t i = 0; i < reprojErrs.size(); ++i) {
				cout << "Projector candidate " << selection.indices[i] + 1
					<< ": final refit per-view RMS=" << reprojErrs[i] << " px" << endl;
			}

			if (!std::isfinite(totalAvgErr) || totalAvgErr > FLAGS_max_projector_rms) {
				cout << "Projector reprojection error above static threshold: " << totalAvgErr << endl;
				return false;
			}

			imagePoints.swap(selectedImagePoints);
			cameraMatrix=fittedMatrix;
			distCoeffs=fittedDistortion;
			objectPoints.swap(selectedObjectPoints);
			frameMeasuredCircleImagePoints.swap(selectedMeasuredPoints);
			frameMeasuredBoardImagePoints.swap(selectedMeasuredBoardPoints);
			camBoardRotations.swap(selectedCameraRotations);
			camBoardTranslations.swap(selectedCameraTranslations);

			saveCameraParams(outputFileName, imageSize,
				1,
				CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5 | CALIB_FIX_K6, cameraMatrix, distCoeffs,
				rvecs,
				tvecs,
				reprojErrs,
				imagePoints,
				totalAvgErr);
			load_calibration_parameters(outputFileName);
			cout << "solving PnP with projector intrinsics for boardRotations and boardTranslations as seen by the projector" << endl;

			cout << "objectPoints size : " << objectPoints.size() << endl;
			cout << "imagePoints size : " << imagePoints.size() << endl;
			cout << "cameraMatrix size : " << cameraMatrix.size() << endl;
			cout << "distCoeffs size : " << distCoeffs.size() << endl;

			Mat rot;
			Mat trans;

			cv::solvePnP(objectPoints.back(), imagePoints.back(),
				cameraMatrix,
				distCoeffs,
				rot, trans);

			boardRotations.clear();
			boardTranslations.clear();
			boardRotations.push_back(rot);
			boardTranslations.push_back(trans);

			return true;
		}
		cout << "Projector final refit failed intrinsic/distortion validation; working model unchanged. Collect clearer, broader views." << endl;
	}
	return false;

}

void Tinker::projector_calibration::setup_projector_parameters(Size _imageSize, string _outputFileName, 
	Size _patternSize, float _squareSize, int _nFramesBeforeDynamicProjectorCalib, Pattern _patternType, float px, float py)
{
	imageSize = _imageSize;
	outputFileName = _outputFileName;
	circlePatternSize = _patternSize;
	squareSize = _squareSize;
	patternType = _patternType;
	patternPosition = Point2f(px, py);

	nFramesBeforeDynamcProjectorCalib = _nFramesBeforeDynamicProjectorCalib;
}

void Tinker::projector_calibration::load_calibration_parameters(string fileName)
{
	struct stat buffer;
	bool found = stat(fileName.c_str(), &buffer) == 0;
	cout << "Camera calibration file exists : " << found << endl;
	if (found) {
		FileStorage fs(fileName, FileStorage::READ);
		Mat loadedMatrix, loadedDistortion;
		fs["camera_matrix"] >> loadedMatrix;
		fs["distortion_coefficients"] >> loadedDistortion;
		string reason;
		if (!valid_projector_distortion(loadedMatrix, loadedDistortion, imageSize, reason)) {
			cout << "Projector calibration load rejected: " << reason << ". Recalibrate." << endl;
			return;
		}
		loadedMatrix.convertTo(cameraMatrix,CV_64F);
		loadedDistortion.convertTo(distCoeffs,CV_64F);
		projector_is_calibrated = true;
	}
}

void Tinker::projector_calibration::reset_boards()
{
	objectPoints.clear();
	imagePoints.clear();
	frameMeasuredCircleImagePoints.clear();
	frameMeasuredBoardImagePoints.clear();
	camBoardRotations.clear();
	camBoardTranslations.clear();
	last_avg_reprojection_error = std::numeric_limits<double>::infinity();
	last_per_view_reprojection_errors.clear();
}

double Tinker::projector_calibration::computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints, const vector<vector<Point2f>>& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat & cameraMatrix, const Mat & distCoeffs, vector<float>& perViewErrors)
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

bool Tinker::projector_calibration::runCalibration(vector<vector<Point2f>> imagePoints, vector<vector<Point3f> > objectPoints, 
	Size imageSize, float aspectRatio, int flags, Mat & cameraMatrix, Mat & distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double & totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5 | CALIB_FIX_K6);
	printf("RMS error reported by calibrateCamera for projector : %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
	string reason;
	if (!ok || !valid_projector_distortion(cameraMatrix, distCoeffs, imageSize, reason)) {
		cout << "Projector distortion fit rejected: " << reason << ". Collect broader, sharper views; RMS alone is insufficient." << endl;
		return false;
	}

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

void Tinker::projector_calibration::saveCameraParams(const string & filename, Size imageSize, float aspectRatio, int flags, const Mat & cameraMatrix, const Mat & distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f>>& imagePoints, double totalAvgErr)
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
