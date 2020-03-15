#include "include\calibration.hpp"

Tinker::calibration::calibration()
{
	camera_calibrator = camera_calibration();
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

bool Tinker::calibration::add_projected(cv::Mat img, cv::Mat processedImg)
{
	vector<cv::Point2f> chessImgPts;

	bool bPrintedPatternFound = camera_calibrator.find_board(img, chessImgPts);

	if (bPrintedPatternFound) {

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
