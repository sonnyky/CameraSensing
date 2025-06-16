#include "include\projector_calibration.hpp"

Tinker::projector_calibration::projector_calibration()
{
	mode = STANDBY;
}

Tinker::projector_calibration::~projector_calibration()
{
}

void Tinker::projector_calibration::load(string projector_config)
{
}

void Tinker::projector_calibration::set_static_candidate_image_points()
{
	candidate_image_points.clear();
	Point2f p;
	//cout << "circlePatternSize : " << circlePatternSize.height << ", " << circlePatternSize.width << endl;
	for (int i = 0; i < circlePatternSize.height; i++) {
		for (int j = 0; j < circlePatternSize.width; j++) {
			p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
			p.y = patternPosition.y + float(i * squareSize);
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

void Tinker::projector_calibration::start_projector_calibration()
{
	if (mode == PROJECTOR_CAPTURING) return;
	imagePoints.clear();
	prevTimestamp = 0;
	delay = 1000;
	mode = PROJECTOR_CAPTURING;
}

bool Tinker::projector_calibration::calibrate()
{
	cout << "mode is : " << mode << endl;
	cout << "imagePoints size : " << imagePoints.size() << endl;

	if (mode != PROJECTOR_CAPTURING) return false;
	if (imagePoints.size() >= (unsigned)nFrames) {
		cout << "got enough points for projector intrinsics calibration." << endl;

		// imagePointsProjObj and objectPoints has to have the same length
		if (imagePoints.size() != objectPoints.size()) {
			cout << "Mismatched sizes. imagePointsProjObj : " << imagePoints.size()
				<< "and objectPoints : "<< objectPoints.size() << endl;
			return false;
		}

		if (runAndSave(outputFileName, imagePoints, objectPoints, imageSize, 1, 0, cameraMatrix, distCoeffs, true, true)) {
			mode = PROJECTOR_CALIBRATED;
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

			boardRotations.push_back(rot);
			boardTranslations.push_back(trans);

			return true;
		}
		else mode = STANDBY;
	}
	return false;

}

void Tinker::projector_calibration::setup_projector_parameters(Size _imageSize, string _outputFileName, 
	Size _patternSize, float _squareSize, Pattern _patternType, float px, float py)
{
	imageSize = _imageSize;
	outputFileName = _outputFileName;
	circlePatternSize = _patternSize;
	squareSize = _squareSize;
	patternType = _patternType;
	patternPosition = Point2f(px, py);
}

void Tinker::projector_calibration::load_calibration_parameters(string fileName)
{
	struct stat buffer;
	bool found = stat(fileName.c_str(), &buffer) == 0;
	cout << "Camera calibration file exists : " << found << endl;
	if (found) {
		projector_is_calibrated = true;
		FileStorage fs(fileName, FileStorage::READ);
		fs["camera_matrix"] >> cameraMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
	}
}

void Tinker::projector_calibration::reset_boards()
{
	objectPoints.clear();
	imagePoints.clear();
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
		distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera for projector : %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

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


bool Tinker::projector_calibration::runAndSave(const string & outputFilename, const vector<vector<Point2f>>& imagePoints, vector<vector<Point3f> > objectPoints, Size imageSize, float aspectRatio, int flags, Mat & _cameraMatrix, Mat & distCoeffs, bool writeExtrinsics, bool writePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, objectPoints, imageSize,
		aspectRatio, flags, _cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if (ok) {
		saveCameraParams(outputFilename, imageSize,
			aspectRatio,
			flags, _cameraMatrix, distCoeffs,
			writeExtrinsics ? rvecs : vector<Mat>(),
			writeExtrinsics ? tvecs : vector<Mat>(),
			writeExtrinsics ? reprojErrs : vector<float>(),
			writePoints ? imagePoints : vector<vector<Point2f> >(),
			totalAvgErr);
	}
	return ok;
}
