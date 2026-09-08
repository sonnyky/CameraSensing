#include <iostream>
#include <sstream>
#include <fstream>
#include <state.hpp>
#include <flags.hpp>
#include "capture.hpp"
#include "calibration.hpp"

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

bool ParseAndCheckCommandLine(int argc, char* argv[]) {
	// ---------------------------Parsing and validation of input args--------------------------------------

	gflags::ParseCommandLineFlags(&argc, &argv, true);
	if (FLAGS_h) {
		showUsage();
		return false;
	}

	std::cout << "Parsing input parameters" << std::endl;

	return true;
}


int main(int argc, char* argv[])
{
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
	Mat projection = Mat(Size(width_second, height_second), CV_8UC1);
#pragma endregion

	try {
		if (!ParseAndCheckCommandLine(argc, argv)) {
			return EXIT_SUCCESS;
		}

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
				if (!calib.is_dynamic_projection_primed()) {
					if (!calib.set_dynamic_projector_image_points(frame, true)) {
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
				if (!calib.set_dynamic_projector_image_points(frame, true)) {
					projImage = cv::Mat::zeros(projImage.size(), projImage.type());
					return;
				}
				calib.draw_projector_pattern(projImage);          // queue that pattern for the next loop

				if (calib.has_dynamic_calibration_solution() &&
					calib.is_dynamic_projector_calibration_satisfied()) {
					std::cout << "Dynamic Projector Calibration complete. Switching to Tracking State." << std::endl;
					capture_state.transition_to<TrackingState>();
				}
			}
			void operator()(TrackingState *  t)
			{
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

		// Printed chessboard on A3 paper
		calibration_manager.setup_camera_calibration_parameters(cv::Size(FLAGS_pattern_width, FLAGS_pattern_height), camera_image_size, FLAGS_pattern_type, 36.0, 1.0, FLAGS_minimum_frames, FLAGS_delay_between_frames, Tinker::DETECTION, FLAGS_write_points, FLAGS_write_extrinsics, 0, FLAGS_camera_filename);

		// printed chessboard on A4 paper
		//calibration_manager.setup_camera_calibration_parameters(cv::Size(FLAGS_pattern_width, FLAGS_pattern_height), camera_image_size, FLAGS_pattern_type, 26.5, 1.0, FLAGS_minimum_frames, FLAGS_delay_between_frames, Tinker::DETECTION, FLAGS_write_points, FLAGS_write_extrinsics, 0, FLAGS_camera_filename);

		calibration_manager.setup_projector_calibration_parameters(Size(1920, 1080), FLAGS_projector_filename, Size(4,5), 120, FLAGS_num_boards_before_dynamic_projector_calib, FLAGS_num_boards_final_projector_calib, Tinker::Pattern::ASYMMETRIC_CIRCLES_GRID, 0, 0);
		calibration_manager.set_projector_static_image_points();
#pragma endregion


#pragma region Capture and processing loop
		int processing = 1;
		do {
			cv::Mat frame = frame_capture.read(); // Capture the frame
			if (!frame.empty()) {
				//cv::imshow("raw", frame);
				cv::imshow("ProjectionWindow", projection);

			}
			else {
				std::cerr << "Error: Empty frame received\n";
				if (cv::waitKey(10) == 27) { // Exit on 'Esc' key
					processing = 0;
				}
				continue;
			}

			std::visit(Visitor{ frame, projection, calibration_manager, capture_state }, capture_state.get_current_state());
			calibration_manager.draw_camera_debug(frame);
			cv::imshow("CameraDebug", frame);

			// Necessary to update the OpenCV window and check for user input
			if (cv::waitKey(10) == 27) { // Exit on 'Esc' key
				processing = 0;
			}
			
		}while (processing == 1);

#pragma endregion Capture and processing loop

	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
