#include "camera_calibration.hpp"
#include "projector_calibration.hpp"

namespace Tinker {

	using system_clock = std::chrono::system_clock;
	//enum { STANDBY = 0, PROJECTOR_CAPTURING = 1, PROJECTOR_CALIBRATED = 2 , DYNAMIC_DETECTION = 3 };
	
	class calibration{
	
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

		void setup_projector_calibration_parameters(Size _imageSize, string _outputFileName, Size _patternSize, float _squareSize,
			int _nFramesBeforeDynamicProjectorCalib, int _nFramesTotalProjectorCalib,
			Pattern _patternType, float px, float py);

		void set_projector_static_image_points();

		bool calibrate_camera(Mat image);
		void switch_to_calibration_mode();

		void load(string cameraConfig, string projectorConfig, string extrinsicsConfig);
		
		// Capture and decide if latest frame is a new frame
		bool accept_new_frame(cv::Mat camMat);


		bool add_projected(cv::Mat img, cv::Mat processedImg);

		const cv::Mat & get_cam_to_proj_rotation() { return rotCamToProj; }
		const cv::Mat & get_cam_to_proj_translation() { return transCamToProj; }
		void loadExtrinsics(string filename, bool absolute = false);

		vector<Point2f> get_projected(const vector<Point3f> & pts,
			const cv::Mat & rotObjToCam,
			const cv::Mat & transObjToCam);
		bool set_dynamic_projector_image_points(cv::Mat img);
		bool is_dynamic_projector_calibration_satisfied() const;
		void reset_dynamic_projection_priming();
		bool is_dynamic_projection_primed() const;
		void set_dynamic_projection_primed(bool primed);
		void reset_dynamic_calibration_solution();
		void mark_dynamic_calibration_solution();
		bool has_dynamic_calibration_solution() const;
		void draw_camera_debug(Mat& image);

		void draw_projector_pattern(Mat& projectorImage);

		Mat process_image_for_circle_detection(Mat img);

		bool calibrate_projector(Mat img);

		void stereo_calibrate();

	private:

		Mat prev_camera_frame;
		double diff_mean;
		std::chrono::time_point<std::chrono::system_clock> last_frame_time;
		double elapsed_time;
		double min_images_diff;
		double min_elapsed_time;

		camera_calibration camera_calibrator;
		projector_calibration projector_calibrator;
		Mat camera_matrix;
		Mat camera_projector_extrinsics;
		Mat projector_matrix;
		bool camera_is_calibrated;

	protected:
		cv::Mat rotCamToProj;
		cv::Mat transCamToProj;

	private:
		bool dynamic_projection_primed = false;
		bool dynamic_calibration_has_solution = false;
		bool has_smoothed_dynamic_board_pose = false;
		cv::Mat smoothed_dynamic_board_rot;
		cv::Mat smoothed_dynamic_board_trans;
	};
}
