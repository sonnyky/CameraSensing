#include "include\calibration.hpp"

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
	int _nFramesTotalProjectorCalib,
	Pattern _patternType, 
	float px, 
	float py
)
{
	projector_calibrator.setup_projector_parameters(_imageSize, _outputFileName, _patternSize, _squareSize, _nFramesBeforeDynamicProjectorCalib, _nFramesTotalProjectorCalib, _patternType, px, py);
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
	double diff_mean = (m[0] + m[1] + m[2]) / 3.0;

	camMat.copyTo(prev_camera_frame);

	using namespace std::chrono;
	std::chrono::time_point<system_clock> latest_frame_time = system_clock::now();
	duration<double> elapsed_seconds = latest_frame_time - last_frame_time;

	//cout << "diff mean: " << diff_mean  << endl;
	double elapsed = elapsed_seconds.count();
	//cout << "elapsed: " << elapsed << endl;

	if ((elapsed > min_elapsed_time) && (diff_mean > min_images_diff)) {
		camMat.copyTo(prev_camera_frame);
		last_frame_time = latest_frame_time;
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

bool Tinker::calibration::set_dynamic_projector_image_points(cv::Mat img)
{
	vector<cv::Point2f> chessImgPts;
	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();

	if (bPrintedPatternFound) {

		drawChessboardCorners(img, camera_calibrator.get_board_size(), Mat(chessImgPts), bPrintedPatternFound);

		cv::Mat boardRot;
		cv::Mat boardTrans;
		camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);

		const auto & camCandObjPts = camera_calibrator.get_candidate_object_points();
		Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
		Point3f axisY = camCandObjPts[camera_calibrator.get_board_size().width] - camCandObjPts[0];
		Point3f pos = camCandObjPts[0] - axisY * (camera_calibrator.get_board_size().width - 2) * 1.3;

		vector<Point3f> auxObjectPoints;
		for (int i = 0; i < projector_calibrator.get_circle_pattern_size().height; i++) {
			for (int j = 0; j < projector_calibrator.get_circle_pattern_size().width; j++) {
				auxObjectPoints.push_back(pos + axisX * float((2 * j) + (i % 2)) + axisY * i);
			}
		}

		Mat Rc1, Tc1, Rc1inv, Tc1inv, Rc2, Tc2, Rp1, Tp1, Rp2, Tp2;
		Rp1 = projector_calibrator.get_board_rotations().back();
		Tp1 = projector_calibrator.get_board_translations().back();
		Rc1 = camera_calibrator.get_board_rotations().back();
		Tc1 = camera_calibrator.get_board_translations().back();
		Rc2 = boardRot;
		Tc2 = boardTrans;

		Mat auxRinv = Mat::eye(3, 3, CV_32F);
		Rodrigues(Rc1, auxRinv);
		auxRinv = auxRinv.inv();
		Rodrigues(auxRinv, Rc1inv);
		Tc1inv = -auxRinv * Tc1;
		Mat Raux, Taux;
		composeRT(Rc2, Tc2, Rc1inv, Tc1inv, Raux, Taux);
		composeRT(Raux, Taux, Rp1, Tp1, Rp2, Tp2);

		vector<Point2f> followingPatternImagePoints;
		if (projector_calibrator.get_camera_matrix().empty()) {
			std::cerr << "projector_calibrator get_camera_matrix are empty!" << std::endl;
		}

		if (projector_calibrator.get_dist_coeffs().empty()) {
			std::cerr << "projector_calibrator get_dist_coeffs are empty!" << std::endl;
		}
		projectPoints(Mat(auxObjectPoints),
			Rp2, Tp2,
			projector_calibrator.get_camera_matrix(),
			projector_calibrator.get_dist_coeffs(),
			followingPatternImagePoints);

		projector_calibrator.set_candidate_image_points(followingPatternImagePoints);
	}
	return bPrintedPatternFound;
}

void Tinker::calibration::draw_projector_pattern(Mat image, Mat projectorImage)
{
	int radius = 20;
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
		cvtColor(img, thresholdedImage, COLOR_RGB2GRAY);
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
	vector<vector<cv::Point2f> > auxImagePointsCamera;

	cout << "check size of all parameters :" << endl;
	cout << "camera_calibrator.get_board_rotations() : " << camera_calibrator.get_board_rotations() .size()<< endl;
	cout << "camera_calibrator.get_board_translations() : " << camera_calibrator.get_board_translations().size() << endl;
	cout << "camera_calibrator.get_camera_matrix() : " << camera_calibrator.get_camera_matrix().size() << endl;
	cout << "camera_calibrator.get_dist_coeffs() : " << camera_calibrator.get_dist_coeffs().size() << endl;

	for (int i = 0; i < objectPoints.size(); i++) {
		vector<cv::Point2f> auxImagePoints;
		projectPoints(cv::Mat(objectPoints[i]),
			camera_calibrator.get_board_rotations()[i],
			camera_calibrator.get_board_translations()[i],
			camera_calibrator.get_camera_matrix(),
			camera_calibrator.get_dist_coeffs(),
			auxImagePoints);

		auxImagePointsCamera.push_back(auxImagePoints);
	}

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

	cv::stereoCalibrate(objectPoints,
		auxImagePointsCamera,
		projector_calibrator.imagePoints,
		cameraMatrix, cameraDistCoeffs,
		projectorMatrix, projectorDistCoeffs,
		camera_calibrator.get_image_size(),
		rotation3x3, transCamToProj,
		essentialMatrix, fundamentalMatrix);

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
		vector<cv::Point2f> circlesImgPts;
		bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, projector_calibrator.get_circle_pattern_size(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);

		if (bProjectedPatternFound) {
			drawChessboardCorners(img, projector_calibrator.get_circle_pattern_size(), Mat(circlesImgPts), bProjectedPatternFound);

			vector<cv::Point3f> circlesObjectPts;
			cv::Mat boardRot;
			cv::Mat boardTrans;

			camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
			camera_calibrator.back_project(boardRot, boardTrans, circlesImgPts, circlesObjectPts);

			camera_calibrator.imagePointsCamObj.push_back(chessImgPts);
			camera_calibrator.get_object_points().push_back(camera_calibrator.get_candidate_object_points());
			camera_calibrator.boardRotations.push_back(boardRot);
			camera_calibrator.boardTranslations.push_back(boardTrans);

			projector_calibrator.imagePoints.push_back(projector_calibrator.get_candidate_image_points());
			projector_calibrator.objectPoints.push_back(circlesObjectPts);

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
