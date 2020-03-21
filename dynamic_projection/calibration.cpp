#include "include\calibration.hpp"

Tinker::calibration::calibration()
{
	camera_is_calibrated = false;
	mode = STANDBY;
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

void Tinker::calibration::setup_projector_calibration_parameters(Size _imageSize, string _outputFileName, Size _patternSize, float _squareSize, 
	Pattern _patternType, float px, float py)
{
	projector_calibrator.setup_projector_parameters(_imageSize, _outputFileName, _patternSize, _squareSize, _patternType, px, py);
}

void Tinker::calibration::set_projector_static_image_points()
{
	projector_calibrator.set_static_candidate_image_points();
}

void Tinker::calibration::calibrate_camera(Mat image)
{
	camera_calibrator.calibrate(image);
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

void Tinker::calibration::loadExtrinsics(string filename, bool absolute)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Rotation_Vector"] >> rotCamToProj;
	fs["Translation_Vector"] >> transCamToProj;
}

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

		cv::Mat boardRot;
		cv::Mat boardTrans;
		camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);

		const auto & camCandObjPts = camera_calibrator.get_candidate_object_points();
		Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
		Point3f axisY = camCandObjPts[camera_calibrator.get_board_size().width] - camCandObjPts[0];
		Point3f pos = camCandObjPts[0] - axisY * (camera_calibrator.get_board_size().width - 2);

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
		projectPoints(Mat(auxObjectPoints),
			Rp2, Tp2,
			projector_calibrator.get_camera_matrix(),
			projector_calibrator.get_dist_coeffs(),
			followingPatternImagePoints);

		projector_calibrator.set_candidate_image_points(followingPatternImagePoints);
	}
	return bPrintedPatternFound;
}

bool Tinker::calibration::set_dynamic_projector_image_points_test(cv::Mat img)
{
	vector<cv::Point2f> chessImgPts;
	bool bPrintedPatternFound = camera_calibrator.find_board(img);
	chessImgPts = camera_calibrator.get_detected_board_points();

	if (bPrintedPatternFound) {

		cv::Mat boardRot;
		cv::Mat boardTrans;
		camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
		drawChessboardCorners(img, camera_calibrator.get_board_size(), Mat(chessImgPts), bPrintedPatternFound);
		Mat Rc1, Tc1, Rc1inv, Tc1inv;
		Rc1 = boardRot;
		Tc1 = boardTrans;
		Mat auxRinv = Mat::eye(3, 3, CV_32F);
		Rodrigues(Rc1, auxRinv);
		auxRinv = auxRinv.inv();
		Rodrigues(auxRinv, Rc1inv);
		Tc1inv = -auxRinv * Tc1;
		vector<Point2f> followingPatternImagePoints;
		projectPoints(Mat(camera_calibrator.get_candidate_object_points()),
			Rc1, Tc1,
			camera_calibrator.get_camera_matrix(),
			camera_calibrator.get_dist_coeffs(),
			followingPatternImagePoints);

		projector_calibrator.set_candidate_image_points(followingPatternImagePoints);
	}

	return false;
}

void Tinker::calibration::draw_projector_pattern(Mat image, Mat projectorImage)
{
	if (mode != PROJECTOR_CAPTURING && mode != PROJECTOR_CALIBRATED) return;
	int radius = 25;

	if (mode == PROJECTOR_CALIBRATED) {
		cout << "checking dynamic points" << endl;
		set_dynamic_projector_image_points(image);
	}
	projectorImage = cv::Mat::zeros(projectorImage.size(), projectorImage.type());
	vector<Point2f> points = projector_calibrator.get_candidate_image_points();
	for (int i = 0; i < points.size(); i++) {
		circle(projectorImage, points[i], radius, CvScalar(255, 255, 255), -1, 8, 0);
	}
	
}

void Tinker::calibration::process_image_for_circle_detection(Mat img)
{

	if (img.type() != CV_8UC1) {
		cvtColor(img, processedImg, CV_RGB2GRAY);
	}
	else {
		processedImg = img;
	}
	cv::threshold(processedImg, processedImg, 210, 255, cv::THRESH_BINARY_INV);
}

void Tinker::calibration::calibrate_projector(Mat img)
{
	if (mode != PROJECTOR_CAPTURING) return;

	process_image_for_circle_detection(img);

	if (add_projected(img, processedImg)) {
		
		projector_calibrator.calibrate();
		set_dynamic_projector_image_points(img);
		mode = PROJECTOR_CALIBRATED;
	}
	imshow("ImageThresholded", processedImg);
}

void Tinker::calibration::stereo_calibrate()
{
	const auto & objectPoints = projector_calibrator.get_object_points();

	vector<vector<cv::Point2f> > auxImagePointsCamera;
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

	cv::stereoCalibrate(objectPoints,
		auxImagePointsCamera,
		projector_calibrator.imagePointsProjObj,
		cameraMatrix, cameraDistCoeffs,
		projectorMatrix, projectorDistCoeffs,
		camera_calibrator.get_image_size(),
		rotation3x3, transCamToProj,
		essentialMatrix, fundamentalMatrix);

	cv::Rodrigues(rotation3x3, rotCamToProj);
}

void Tinker::calibration::start_projector_calibration()
{
	mode = PROJECTOR_CAPTURING;
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

			vector<cv::Point3f> circlesObjectPts;
			cv::Mat boardRot;
			cv::Mat boardTrans;

			camera_calibrator.compute_candidate_board_pose(chessImgPts, boardRot, boardTrans);
			camera_calibrator.back_project(boardRot, boardTrans, circlesImgPts, circlesObjectPts);

			camera_calibrator.imagePointsCamObj.push_back(chessImgPts);
			camera_calibrator.get_object_points().push_back(camera_calibrator.get_candidate_object_points());
			camera_calibrator.get_board_rotations().push_back(boardRot);
			camera_calibrator.get_board_translations().push_back(boardTrans);

			projector_calibrator.imagePointsProjObj.push_back(projector_calibrator.get_candidate_image_points());
			projector_calibrator.get_object_points().push_back(circlesObjectPts);

			return true;
		}
	}
	return false;
}
