#include <iostream>
#include <sstream>
#include <fstream>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <cmath>
#include <windows.h>
#include <state.hpp>
#include <flags.hpp>
#include "capture.hpp"
#include "calibration.hpp"

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

std::filesystem::path resolve_calibration_output_directory()
{
	std::filesystem::path outputDirectory;
	if (FLAGS_calibration_output_dir.empty()) {
		std::array<wchar_t, 32768> executablePath{};
		const DWORD length = GetModuleFileNameW(nullptr, executablePath.data(),
			static_cast<DWORD>(executablePath.size()));
		if (length == 0 || length == executablePath.size()) {
			throw std::runtime_error("Unable to determine the executable directory for calibration output.");
		}
		outputDirectory = std::filesystem::path(std::wstring(executablePath.data(), length)).parent_path();
	}
	else {
		outputDirectory = FLAGS_calibration_output_dir;
	}

	std::error_code error;
	std::filesystem::create_directories(outputDirectory, error);
	if (error) {
		throw std::runtime_error("Unable to create calibration output directory: " + outputDirectory.string());
	}
	return outputDirectory;
}

bool ParseAndCheckCommandLine(int argc, char* argv[]) {
	// ---------------------------Parsing and validation of input args--------------------------------------

	gflags::SetUsageMessage("Calibrate a camera/projector pair using a planar chessboard and projected circles.");
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	if (FLAGS_h) {
		showUsage();
		return false;
	}
	if (!std::isfinite(FLAGS_board_square_size_cm) || FLAGS_board_square_size_cm <= 0.0) {
		throw std::invalid_argument("--board_square_size_cm must be a finite positive length in centimeters.");
	}
	if (!std::isfinite(FLAGS_static_projector_spacing_px) || FLAGS_static_projector_spacing_px <= 0.0 ||
		!std::isfinite(FLAGS_static_projector_center_x) || FLAGS_static_projector_center_x < 0.0 || FLAGS_static_projector_center_x > 1.0 ||
		!std::isfinite(FLAGS_static_projector_center_y) || FLAGS_static_projector_center_y < 0.0 || FLAGS_static_projector_center_y > 1.0 ||
		FLAGS_projected_circle_radius == 0 ||
		2.0 * FLAGS_projected_circle_radius >= std::sqrt(2.0) * FLAGS_static_projector_spacing_px) {
		throw std::invalid_argument("Static projector spacing/center must be finite and valid; circle radius must be positive and circles must not touch.");
	}
	for (double span : { FLAGS_minimum_projector_board_position_span, FLAGS_minimum_projector_pattern_span }) {
		if (!std::isfinite(span) || span <= 0.0 || span > std::sqrt(2.0)) {
			throw std::invalid_argument("Projector coverage spans must be finite, positive, and no larger than sqrt(2).");
		}
	}

	std::cout << "Parsing input parameters" << std::endl;
	if (FLAGS_board_aruco_dictionary < 0 || FLAGS_board_aruco_dictionary > 21 ||
		FLAGS_board_origin_aruco_id < 0 || FLAGS_board_origin_secondary_aruco_id < 0 ||
		FLAGS_board_origin_aruco_id == FLAGS_board_origin_secondary_aruco_id) {
		throw std::invalid_argument("ArUco dictionary must be 0..21 and the two origin IDs must be distinct nonnegative IDs.");
	}
	for (double value : {FLAGS_board_origin_marker_x_squares, FLAGS_board_origin_marker_y_squares,
		FLAGS_board_secondary_marker_x_squares, FLAGS_board_secondary_marker_y_squares,
		FLAGS_projection_region_top_cm, FLAGS_projection_region_center_x_cm}) {
		if (!std::isfinite(value)) throw std::invalid_argument("Marker locations and projection region offsets must be finite.");
	}
	for (double value : {FLAGS_board_origin_marker_tolerance_squares, FLAGS_board_tracking_max_motion_squares,
		FLAGS_projection_region_width_cm, FLAGS_projection_region_height_cm}) {
		if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Orientation tolerances and projection region dimensions must be finite positive values.");
	}
	if (FLAGS_projector_settle_ms == 0 || FLAGS_projector_settle_ms > 5000 || FLAGS_projector_discard_frames > 30) {
		throw std::invalid_argument("Projector settling must be 1..5000 ms and discarded frame count 0..30.");
	}
	if (!std::isfinite(FLAGS_projector_detection_threshold) ||
		(FLAGS_projector_detection_threshold != -1 && (FLAGS_projector_detection_threshold < 0 || FLAGS_projector_detection_threshold > 255))) {
		throw std::invalid_argument("--projector_detection_threshold must be -1 (Otsu) or finite within 0..255.");
	}
	if (!std::isfinite(FLAGS_projector_blob_min_area) || !std::isfinite(FLAGS_projector_blob_max_area) ||
		FLAGS_projector_blob_min_area <= 0 || FLAGS_projector_blob_max_area <= FLAGS_projector_blob_min_area) {
		throw std::invalid_argument("Projector blob areas must be finite and satisfy 0 < min_area < max_area.");
	}
	for (double value : { FLAGS_projector_blob_min_circularity, FLAGS_projector_blob_min_inertia, FLAGS_projector_blob_min_convexity }) {
		if (!std::isfinite(value) || value < 0 || value > 1) {
			throw std::invalid_argument("Projector blob shape filters must be finite within 0..1; zero disables a filter.");
		}
	}

	return true;
}


int main(int argc, char* argv[])
{
	try {
		if (!ParseAndCheckCommandLine(argc, argv)) return EXIT_SUCCESS;
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
	int TRACKING_MODE = 0;
	int CALIBRATION_MODE = 1;
	int MODE = 0;

#pragma region opencv window
	int width_first = 1920;
	int height_first = 0;

	// define dimension of the second display
	int width_second = 1920;
	int height_second = 1080;

	// move the window to the second display 
	// (assuming the two displays are top aligned)
	namedWindow("ProjectionWindow", WND_PROP_FULLSCREEN);
	moveWindow("ProjectionWindow", width_first, height_first);
	setWindowProperty("ProjectionWindow", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	namedWindow("CameraDebug", WINDOW_NORMAL);
	moveWindow("CameraDebug", 0, 0);
	resizeWindow("CameraDebug", 640, 360);
	namedWindow("ImageThresholded", WINDOW_NORMAL);
	moveWindow("ImageThresholded", 980, 0);
	resizeWindow("ImageThresholded", 640, 360);

	// create target image
	Mat detectionResized = Mat(Size(width_second, height_second), CV_8UC1);
	Mat projection = Mat::zeros(Size(width_second, height_second), CV_8UC1);
#pragma endregion

	try {
		const std::filesystem::path calibrationOutputDirectory = resolve_calibration_output_directory();
		const std::string cameraOutputFilename = (calibrationOutputDirectory / "camera_params.xml").string();
		const std::string projectorOutputFilename = (calibrationOutputDirectory / "projector_params.xml").string();
		std::cout << "Calibration results will be written to " << calibrationOutputDirectory.string() << std::endl;

#pragma region states initializations
		CaptureStateManager capture_state;

		struct Visitor
		{
			cv::Mat& frame;
			cv::Mat& projImage;
			Tinker::calibration& calib;
			CaptureStateManager& capture_state;

			Visitor(cv::Mat& f, cv::Mat& pImg, Tinker::calibration& c, CaptureStateManager& s) : frame(f), projImage(pImg), calib(c), capture_state(s) {}

			void operator()(IdleState* t)
			{
				cout << " in idle state " << endl;
			}
			void operator()(CameraCalibrationState* c)
			{
				bool success = calib.calibrate_camera(frame);
				if (success) {
					std::cout << "Camera Calibration complete. Switching to Static Projector Calibration State." << std::endl;
					calib.reset_sample_capture_gate();
					capture_state.transition_to<StaticProjectorCalibrationState>();
				}
			}
			void operator()(StaticProjectorCalibrationState* c)
			{
				calib.draw_projector_pattern(projImage);
				bool success = calib.calibrate_projector(frame);
				if (success) {
					std::cout << "Static Projector Calibration complete. Switching to Dynamic Projector Calibration State." << std::endl;
					calib.reset_dynamic_projection_priming();
					calib.reset_dynamic_calibration_solution();
					calib.reset_sample_capture_gate();
					capture_state.transition_to<DynamicProjectorCalibrationState>();
				}
			}
			void operator()(DynamicProjectorCalibrationState* c)
			{
				// Calibration adds debug overlays to frame. Projection pose detection
				// must use the original pixels from this same capture instead.
				const cv::Mat projectionFrame = frame.clone();
				if (!calib.is_dynamic_projection_primed()) {
					if (!calib.set_dynamic_projector_image_points(projectionFrame, true)) {
						projImage = cv::Mat::zeros(projImage.size(), projImage.type());
						return;
					}
					calib.draw_projector_pattern(projImage);
					calib.set_dynamic_projection_primed(true);
					return;
				}

				bool success = calib.calibrate_projector(frame);  // process the frame corresponding to the pattern already on screen
				if (success) {
					calib.mark_dynamic_calibration_solution();
				}
				if (!calib.set_dynamic_projector_image_points(projectionFrame, true)) {
					projImage = cv::Mat::zeros(projImage.size(), projImage.type());
					return;
				}
				calib.draw_projector_pattern(projImage);          // queue that pattern for the next loop

				if (calib.has_dynamic_calibration_solution() &&
					calib.is_dynamic_projector_calibration_satisfied()) {
					calib.save_stereo_calibration();
					std::cout << "Dynamic Projector Calibration complete. Switching to Tracking State." << std::endl;
					capture_state.transition_to<TrackingState>();
				}
			}
			void operator()(TrackingState *  t)
			{
				// Same physical region as dynamic calibration; false enables smoothing only.
				if (!calib.set_dynamic_projector_image_points(frame, false)) {
					projImage = cv::Mat::zeros(projImage.size(), projImage.type());
					return;
				}
				calib.draw_projector_pattern(projImage);
			}
		};

		Tinker::capture frame_capture("webcam", 0);
		//cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);

		capture_state.transition_to<CameraCalibrationState>();
	
#pragma endregion


#pragma region calibration parameters settings and calibration object instantiation
		
		Tinker::calibration calibration_manager;
		cv::Size camera_image_size(1920, 1080);
		{
			cv::Mat first_frame = frame_capture.read();
			if (!first_frame.empty()) {
				camera_image_size = first_frame.size();
			}
		}

		// Convert the printed square side length from centimeters to millimeters.
		const float boardSquareSizeMm = static_cast<float>(FLAGS_board_square_size_cm * 10.0);
		calibration_manager.setup_camera_calibration_parameters(cv::Size(FLAGS_pattern_width, FLAGS_pattern_height), camera_image_size, boardSquareSizeMm, 1.0, FLAGS_minimum_frames, FLAGS_delay_between_frames, Tinker::DETECTION, 0, cameraOutputFilename);

		calibration_manager.setup_projector_calibration_parameters(Size(1920, 1080), projectorOutputFilename, Size(4,5), static_cast<float>(FLAGS_static_projector_spacing_px), FLAGS_num_boards_before_dynamic_projector_calib, FLAGS_num_boards_final_projector_calib, Tinker::Pattern::ASYMMETRIC_CIRCLES_GRID, 0, 0);
		calibration_manager.set_projector_static_image_points();
#pragma endregion


#pragma region Capture and processing loop
		int processing = 1;
		do {
			cv::imshow("ProjectionWindow", projection);
			if (cv::waitKey(static_cast<int>(FLAGS_projector_settle_ms)) == 27) break;
			calibration_manager.record_displayed_projector_pattern(cv::countNonZero(projection) > 0);
			for (uint64_t skipped = 0; skipped < FLAGS_projector_discard_frames; ++skipped) frame_capture.read();
			cv::Mat frame = frame_capture.read(); // Capture the frame
			if (frame.empty()) {
				std::cerr << "Error: Empty frame received\n";
				if (cv::waitKey(10) == 27) { // Exit on 'Esc' key
					processing = 0;
				}
				continue;
			}

			cv::Mat debugFrame = frame.clone();
			std::visit(Visitor{ frame, projection, calibration_manager, capture_state }, capture_state.get_current_state());
			calibration_manager.draw_camera_debug(debugFrame);
			cv::imshow("CameraDebug", debugFrame);

			// Necessary to update the OpenCV window and check for user input
			if (cv::waitKey(10) == 27) { // Exit on 'Esc' key
				processing = 0;
			}
			
		}while (processing == 1);

#pragma endregion Capture and processing loop

	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
