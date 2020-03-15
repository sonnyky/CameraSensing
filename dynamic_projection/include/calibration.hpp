#include "camera_calibration.hpp"
#include "projector_calibration.hpp"

namespace Tinker {
	class calibration {
	public:
		calibration();
		~calibration();

		void setup_camera_calibration_parameters(
			Size boardSize_,
			Size imageSize_,
			string pattern_,
			float squareSize_,
			float aspectRatio_,
			int nFrames_,
			int delay_,
			int mode_,
			bool writePoints_,
			bool writeExtrinsics_,
			int cameraId_,
			string outputFileName_);

		void calibrate_camera(Mat image);
		void switch_to_calibration_mode();

		void load(string cameraConfig, string projectorConfig, string extrinsicsConfig);
		
		void draw_projector_points(Mat image);

		void detect_circles_from_projector(Mat image);

		bool add_projected(cv::Mat img, cv::Mat processedImg);

		const cv::Mat & get_cam_to_proj_rotation() { return rotCamToProj; }
		const cv::Mat & get_cam_to_proj_translation() { return transCamToProj; }
		void loadExtrinsics(string filename, bool absolute = false);

		vector<Point2f> get_projected(const vector<Point3f> & pts,
			const cv::Mat & rotObjToCam,
			const cv::Mat & transObjToCam);

	private:
		camera_calibration camera_calibrator;
		projector_calibration projector_calibrator;
		Mat camera_matrix;
		Mat camera_projector_extrinsics;
		Mat projector_matrix;
		bool camera_is_calibrated;

	protected:
		cv::Mat rotCamToProj;
		cv::Mat transCamToProj;
	};
}