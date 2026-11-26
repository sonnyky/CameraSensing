#include <iostream>
#include <sstream>
#include <fstream>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <cmath>
#include <vector>
#include <algorithm>
#include <windows.h>
#include <state.hpp>
#include <flags.hpp>
#include "capture.hpp"
#include "calibration.hpp"
#include "display_selection.hpp"

namespace {
struct WindowsDisplay {
	Tinker::DisplayGeometry geometry;
	std::wstring device;
};

BOOL CALLBACK collect_display(HMONITOR monitor, HDC, LPRECT, LPARAM data)
{
	auto& displays=*reinterpret_cast<std::vector<WindowsDisplay>*>(data);
	MONITORINFOEXW info{};
	info.cbSize=sizeof(info);
	if (!GetMonitorInfoW(monitor,&info)) return TRUE;
	displays.push_back({{info.rcMonitor.left,info.rcMonitor.top,
		info.rcMonitor.right-info.rcMonitor.left,info.rcMonitor.bottom-info.rcMonitor.top,
		(info.dwFlags & MONITORINFOF_PRIMARY)!=0},info.szDevice});
	return TRUE;
}
}

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
	if (FLAGS_calibration_file_mode!="redo" && FLAGS_calibration_file_mode!="reuse")
		throw std::invalid_argument("--calibration_file_mode must be redo or reuse.");
	if (FLAGS_projector_display_index < -1) {
		throw std::invalid_argument("--projector_display_index must be -1 or nonnegative.");
	}
	if (FLAGS_camera_width == 0 || FLAGS_camera_height == 0 || FLAGS_camera_width > 16384 || FLAGS_camera_height > 16384) {
		throw std::invalid_argument("Camera width and height must be within 1..16384 pixels.");
	}
	if (FLAGS_projected_circle_radius == 0) throw std::invalid_argument("Projected circle radius must be positive.");
	if (!std::isfinite(FLAGS_static_gray_max_board_motion_px) || FLAGS_static_gray_max_board_motion_px<=0 ||
		!std::isfinite(FLAGS_static_gray_min_direct_contrast) || FLAGS_static_gray_min_direct_contrast<=0 || FLAGS_static_gray_min_direct_contrast>255 ||
		!std::isfinite(FLAGS_static_gray_min_bit_contrast) || FLAGS_static_gray_min_bit_contrast<=0 || FLAGS_static_gray_min_bit_contrast>255 ||
		FLAGS_static_gray_sample_radius_px>10 || FLAGS_static_gray_projector_block_size<1 ||
		FLAGS_static_gray_projector_block_size>32 || FLAGS_static_gray_phase_period_px<FLAGS_static_gray_projector_block_size ||
		FLAGS_static_gray_phase_period_px>256 || FLAGS_static_gray_phase_period_px%FLAGS_static_gray_projector_block_size!=0 ||
		FLAGS_static_gray_start_delay_seconds>30) {
		throw std::invalid_argument("Static Gray-code motion/contrast settings are invalid.");
	}
	for (double span : { FLAGS_minimum_projector_board_position_span, FLAGS_minimum_projector_pattern_span }) {
		if (!std::isfinite(span) || span <= 0.0 || span > std::sqrt(2.0)) {
			throw std::invalid_argument("Projector coverage spans must be finite, positive, and no larger than sqrt(2).");
		}
	}

	std::cout << "Parsing input parameters" << std::endl;
	if (FLAGS_board_aruco_dictionary < 0 || FLAGS_board_aruco_dictionary > 21 ||
		FLAGS_board_origin_aruco_id < 0) {
		throw std::invalid_argument("ArUco dictionary must be 0..21 and the origin ID must be nonnegative.");
	}
	for (double value : {FLAGS_projection_region_top_cm, FLAGS_projection_region_center_x_cm}) {
		if (!std::isfinite(value)) throw std::invalid_argument("Projection region offsets must be finite.");
	}
	for (double value : {FLAGS_board_tracking_max_motion_squares,
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
	SetProcessDPIAware();
	std::vector<WindowsDisplay> detectedDisplays;
	if (!EnumDisplayMonitors(nullptr,nullptr,collect_display,reinterpret_cast<LPARAM>(&detectedDisplays))) {
		std::cerr << "Unable to enumerate Windows displays." << std::endl;
		return EXIT_FAILURE;
	}
	std::vector<Tinker::DisplayGeometry> displayGeometry;
	for (size_t index=0;index<detectedDisplays.size();++index) {
		const auto& display=detectedDisplays[index];
		displayGeometry.push_back(display.geometry);
		std::wcout << L"Display " << index << L": " << display.device << L" at (" << display.geometry.left << L','
			<< display.geometry.top << L") " << display.geometry.width << L'x' << display.geometry.height
			<< (display.geometry.primary ? L" [primary]" : L"") << std::endl;
	}
	size_t projectorDisplayIndex=0;
	try {
		projectorDisplayIndex=Tinker::select_projector_display(displayGeometry,FLAGS_projector_display_index);
	} catch (const std::exception& error) {
		std::cerr << error.what() << std::endl;
		return EXIT_FAILURE;
	}
	const auto projectorDisplay=displayGeometry[projectorDisplayIndex];
	const int width_second=projectorDisplay.width;
	const int height_second=projectorDisplay.height;
	std::cout << "ProjectionWindow target: display " << projectorDisplayIndex << " at (" << projectorDisplay.left << ','
		<< projectorDisplay.top << ") " << width_second << 'x' << height_second << std::endl;

	namedWindow("ProjectionWindow", WINDOW_NORMAL);
	moveWindow("ProjectionWindow", projectorDisplay.left, projectorDisplay.top);
	resizeWindow("ProjectionWindow",width_second,height_second);
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
					std::cout << "Camera Calibration complete." << std::endl;
					projImage=cv::Mat::zeros(projImage.size(),projImage.type());
					cv::imshow("ProjectionWindow",projImage);
					for (uint64_t remaining=FLAGS_static_gray_start_delay_seconds;remaining>0;--remaining) {
						std::cout << "Static projector calibration starts in " << remaining << "..." << std::endl;
						cv::Mat countdownFrame=frame.clone();
						const std::string message="Static projector calibration starts in " + std::to_string(remaining);
						int baseline=0;
						const cv::Size textSize=cv::getTextSize(message,cv::FONT_HERSHEY_SIMPLEX,1.1,2,&baseline);
						const cv::Point textOrigin(std::max(10,(countdownFrame.cols-textSize.width)/2),
							std::max(textSize.height+10,(countdownFrame.rows+textSize.height)/2));
						cv::putText(countdownFrame,message,textOrigin,cv::FONT_HERSHEY_SIMPLEX,1.1,cv::Scalar(0,255,255),2,cv::LINE_AA);
						cv::imshow("CameraDebug",countdownFrame);
						if (cv::waitKey(1000)==27) throw std::runtime_error("Calibration cancelled during static-projector countdown.");
					}
					std::cout << "Switching to Static Projector Calibration State." << std::endl;
					calib.reset_sample_capture_gate();
					calib.reset_static_gray_code(projImage);
					capture_state.transition_to<StaticProjectorCalibrationState>();
				}
			}
			void operator()(StaticProjectorCalibrationState* c)
			{
				bool success = calib.process_static_gray_code_frame(frame,projImage);
				if (success) {
					calib.save_stereo_calibration(true);
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

		Tinker::capture frame_capture("webcam", 0,static_cast<int>(FLAGS_camera_width),static_cast<int>(FLAGS_camera_height));
		//cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);

#pragma endregion


#pragma region calibration parameters settings and calibration object instantiation
		
		Tinker::calibration calibration_manager;
		cv::Size camera_image_size(1920, 1080);
		{
			cv::Mat first_frame = frame_capture.read();
			if (!first_frame.empty()) {
				camera_image_size = first_frame.size();
				std::cout << "Actual camera frame size: " << camera_image_size.width << 'x' << camera_image_size.height << std::endl;
				if (camera_image_size.width != FLAGS_camera_width || camera_image_size.height != FLAGS_camera_height)
					std::cerr << "Warning: camera did not negotiate the requested " << FLAGS_camera_width << 'x' << FLAGS_camera_height << " mode." << std::endl;
			}
		}

		// Convert the printed square side length from centimeters to millimeters.
		const float boardSquareSizeMm = static_cast<float>(FLAGS_board_square_size_cm * 10.0);
		calibration_manager.setup_camera_calibration_parameters(cv::Size(FLAGS_pattern_width, FLAGS_pattern_height), camera_image_size, boardSquareSizeMm, 1.0, FLAGS_minimum_frames, FLAGS_delay_between_frames, Tinker::DETECTION, 0, cameraOutputFilename);

		calibration_manager.setup_projector_calibration_parameters(Size(width_second, height_second), projectorOutputFilename, Size(4,5), 1.0f, FLAGS_num_boards_before_dynamic_projector_calib, FLAGS_num_boards_final_projector_calib, Tinker::Pattern::ASYMMETRIC_CIRCLES_GRID, 0, 0);

		if (FLAGS_calibration_file_mode=="reuse") {
			const auto reuseStage=calibration_manager.load_saved_calibration(projectorOutputFilename);
			if (reuseStage==Tinker::CalibrationReuseStage::Complete) {
				capture_state.transition_to<TrackingState>();
			}
			else if (reuseStage==Tinker::CalibrationReuseStage::StaticProjector) {
				calibration_manager.reset_dynamic_projection_priming();
				calibration_manager.reset_dynamic_calibration_solution();
				calibration_manager.reset_sample_capture_gate();
				capture_state.transition_to<DynamicProjectorCalibrationState>();
			}
			else if (reuseStage==Tinker::CalibrationReuseStage::CameraOnly) {
				calibration_manager.reset_sample_capture_gate();
				calibration_manager.reset_static_gray_code(projection);
				capture_state.transition_to<StaticProjectorCalibrationState>();
			}
			else capture_state.transition_to<CameraCalibrationState>();
		}
		else {
			std::cout << "Calibration file mode: redo; saved calibration files will be replaced after successful phases." << std::endl;
			capture_state.transition_to<CameraCalibrationState>();
		}
#pragma endregion


#pragma region Capture and processing loop
		int processing = 1;
		do {
			cv::imshow("ProjectionWindow", projection);
			const int settleKey=cv::waitKey(static_cast<int>(FLAGS_projector_settle_ms));
			if (settleKey == 27) break;
			if (settleKey == ' ' || settleKey == 13 || settleKey == 10)
				calibration_manager.request_static_gray_capture();
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
			const int inputKey=cv::waitKey(10);
			if (inputKey == ' ' || inputKey == 13 || inputKey == 10)
				calibration_manager.request_static_gray_capture();
			if (inputKey == 27) { // Exit on 'Esc' key
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
