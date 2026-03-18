#include "camera_calibration.hpp"
#include <algorithm>

Tinker::camera_calibration::camera_calibration()
{

}

Tinker::camera_calibration::~camera_calibration()
{
	calibrationStatus = CAPTURING;
}

void Tinker::camera_calibration::setup_parameters(cv::Size boardSize_, cv::Size imageSize_, string pattern_, float squareSize_, float aspectRatio_, int nFrames_, int delay_, int mode_, bool writePoints_, bool writeExtrinsics_, int cameraId_, std::string outputFileName_)
{
	boardSize = boardSize_;
	imageSize = imageSize_;
	pattern = pattern_;
	patternLengthInRealUnits = squareSize_;
	aspectRatio = aspectRatio_;
	nframes = nFrames_;
	delay = delay_;
	calibrationStatus = mode_;
	writePoints = writePoints_;
	writeExtrinsics = writeExtrinsics_;
	cameraId = cameraId_;
	outputFilename = outputFileName_;
	load_camera_matrix(outputFileName_);
}

bool Tinker::camera_calibration::calibrate(Mat image_)
{
	Mat viewGray;

	Pattern calibPattern = CHESSBOARD;
	if (pattern == "circles_grid") {
		calibPattern = CIRCLES_GRID;
	}
	else if (pattern == "asymmetric_circles_grid") {
		calibPattern = ASYMMETRIC_CIRCLES_GRID;
	}
	switch (calibPattern)
	{
	case CHESSBOARD:
		found = findChessboardCorners(image_, boardSize, pointbuf,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		break;
	case CIRCLES_GRID:
		found = findCirclesGrid(image_, boardSize, pointbuf);
		break;
	case ASYMMETRIC_CIRCLES_GRID:
		found = findCirclesGrid(image_, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
		break;
	default:
		break;
	}

	cvtColor(image_, viewGray, COLOR_BGR2GRAY);
	// improve the found corners' coordinate accuracy
	if (calibPattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(11, 11),
		Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

	if (found && (clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC))
	{
		imagePoints.push_back(pointbuf);
		prevTimestamp = clock();
	}

	// Once we have more points than the threshold we run calibration on the aggregated points to get a global model
	if (imagePoints.size() >= (unsigned)nframes)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> perViewRms;
		double totalAvgErr = 0;

		// here cameraMatrix and distCoeffs are the calibration results from the aggregated image points
		bool calibSuccess = runCalibration(imagePoints, imageSize, boardSize, calibPattern, patternLengthInRealUnits,
			aspectRatio, flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, perViewRms, totalAvgErr);
		printf("%s. avg reprojection error for this batch = %.2f\n",
			calibSuccess ? "Calibration succeeded" : "Calibration failed",
			totalAvgErr);

		// Keep the best views by reprojection error instead of dropping everything
		// above a hard threshold, which can wipe the capture set and block progress.
		std::vector<std::pair<float, size_t>> rankedViews;
		rankedViews.reserve(perViewRms.size());
		for (size_t i = 0; i < perViewRms.size(); ++i) {
			rankedViews.emplace_back(perViewRms[i], i);
		}
		std::sort(rankedViews.begin(), rankedViews.end(),
			[](const auto& a, const auto& b) {
				return a.first < b.first;
			});

		const size_t keepCount = std::min(
			imagePoints.size(),
			static_cast<size_t>(std::max(nframes, 12)));

		std::vector<std::vector<cv::Point2f>> filteredImagePoints;
		std::vector<float> filteredRms;
		filteredImagePoints.reserve(keepCount);
		filteredRms.reserve(keepCount);

		for (size_t i = 0; i < keepCount && i < rankedViews.size(); ++i) {
			const size_t viewIndex = rankedViews[i].second;
			filteredImagePoints.push_back(imagePoints[viewIndex]);
			filteredRms.push_back(rankedViews[i].first);
		}

		imagePoints.swap(filteredImagePoints);
		perViewRms.swap(filteredRms);

		// if we still have enough points after removing high rms views then we run the calibration one more time for the final camera values
		// if we don't have enouhgh points then we start taking views again
		if (imagePoints.size() >= (unsigned)nframes)
		{
			bool calibSuccess = runCalibration(imagePoints, imageSize, boardSize, calibPattern, patternLengthInRealUnits,
				aspectRatio, flags, cameraMatrix, distCoeffs,
				rvecs, tvecs, perViewRms, totalAvgErr);
			printf("%s. avg reprojection error for this batch = %.2f\n",
				calibSuccess ? "Final camera Calibration succeeded" : "Final Camera Calibration failed",
				totalAvgErr);

			saveCameraParams(outputFilename, imageSize,
				boardSize, patternLengthInRealUnits, aspectRatio,
				flags, cameraMatrix, distCoeffs,
				writeExtrinsics ? rvecs : vector<Mat>(),
				writeExtrinsics ? tvecs : vector<Mat>(),
				writeExtrinsics ? perViewRms : vector<float>(),
				writePoints ? imagePoints : vector<vector<Point2f> >(),
				totalAvgErr);
			load_camera_matrix(outputFilename);
			return true;
		}
		else {
			return false;
		}		
	}
	else {
		cout << "more points needed. we currently have : " << imagePoints.size() << " points." << endl;
	}

	return false;

}

void Tinker::camera_calibration::set_to_calibration_mode()
{
	imagePoints.clear();
	prevTimestamp = 0;
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

	bool foundChessCorners = findChessboardCorners(gray, boardSize, detected_board_points,
		CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

	if (foundChessCorners) {
		cornerSubPix(gray, detected_board_points, Size(11, 11),
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	}

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
	if (imgPt.size() == 0) {
		return false;
	}
	else
	{
		Mat imgPt_h = Mat::zeros(3, imgPt.size(), CV_32F);
		for (int h = 0; h < imgPt.size(); ++h) {
			imgPt_h.at<float>(0, h) = imgPt[h].x;
			imgPt_h.at<float>(1, h) = imgPt[h].y;
			imgPt_h.at<float>(2, h) = 1.0f;
		}
		Mat Kinv64 = cameraMatrix.inv();
		Mat Kinv, boardRot, boardTrans;
		Kinv64.convertTo(Kinv, CV_32F);
		boardRot64.convertTo(boardRot, CV_32F);
		boardTrans64.convertTo(boardTrans, CV_32F);

		// Transform all image points to world points in camera reference frame
		// and then into the plane reference frame
		Mat worldImgPt = Mat::zeros(3, imgPt.size(), CV_32F);
		Mat rot3x3;
		Rodrigues(boardRot, rot3x3);

		Mat transPlaneToCam = rot3x3.inv()*boardTrans;

		for (int i = 0; i < imgPt.size(); ++i) {
			Mat col = imgPt_h.col(i);
			Mat worldPtcam = Kinv * col;
			Mat worldPtPlane = rot3x3.inv()*(worldPtcam);

			float scale = transPlaneToCam.at<float>(2) / worldPtPlane.at<float>(2);
			Mat worldPtPlaneReproject = scale * worldPtPlane - transPlaneToCam;

			Point3f pt;
			pt.x = worldPtPlaneReproject.at<float>(0);
			pt.y = worldPtPlaneReproject.at<float>(1);
			pt.z = 0;
			worldPt.push_back(pt);
		}
	}
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
