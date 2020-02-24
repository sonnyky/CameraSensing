#include "camera_calibration.hpp"

namespace Tinker {
	class calibration {
	public:
		calibration();
		~calibration();

		void setup_camera_calibration_parameters(
			Size boardSize_,
			Size imageSize_,
			float squareSize_,
			float aspectRatio_,
			int nFrames_,
			int delay_,
			int mode_,
			bool writePoints_,
			bool writeExtrinsics_,
			int cameraId_,
			string outputFileName_);

	private:
		camera_calibration camera_calibrator;
	};
}