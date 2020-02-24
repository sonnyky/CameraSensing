#include "include\calibration.hpp"

Tinker::calibration::calibration()
{
	camera_calibrator = camera_calibration();
}

Tinker::calibration::~calibration()
{
}

void Tinker::calibration::setup_camera_calibration_parameters(Size boardSize_, Size imageSize_, float squareSize_, float aspectRatio_, int nFrames_, int delay_, int mode_, bool writePoints_, bool writeExtrinsics_, int cameraId_, string outputFileName_)
{
	camera_calibrator.setup_parameters(
		boardSize_,
		imageSize_,
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
}
