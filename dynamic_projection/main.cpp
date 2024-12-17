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

	gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
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

	try {
		if (!ParseAndCheckCommandLine(argc, argv)) {
			return EXIT_SUCCESS;
		}

#pragma region states initializations
		CaptureStateManager capture_state;

		struct Visitor
		{
			void operator()(IdleState* t)
			{
				cout << " in idle state " << endl;
			}
			void operator()(TrackingState *  t)
			{
				cout<< " in tracking state "<< endl;
			}
			void operator()(CalibrationState * c)
			{
				cout << " in calibration state " << endl;
			}
		};

		std::variant<IdleState *, TrackingState * , CalibrationState * > v = capture_state.get_current_state<std::variant<IdleState, TrackingState, CalibrationState>>();
		std::visit(Visitor{}, v);
		
		Tinker::capture frame_capture("webcam", 0);
		cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);

		capture_state.transition_to<CalibrationState>();
		v = capture_state.get_current_state<std::variant<IdleState, TrackingState, CalibrationState>>();
		std::visit(Visitor{}, v);


#pragma endregion

#pragma region opencv window
		int width_first = 1920;
		int height_first = 0;

		// define dimension of the second display
		int width_second = 1920;
		int height_second = 1080;

		// move the window to the second display 
		// (assuming the two displays are top aligned)
		//namedWindow("ProjectionWindow", WND_PROP_FULLSCREEN);
		//moveWindow("ProjectionWindow", width_first, height_first);
		//setWindowProperty("ProjectionWindow", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

		// create target image
		Mat detectionResized = Mat(Size(width_second, height_second), CV_8UC1);
		Mat projection = Mat(Size(width_second, height_second), CV_8UC1);
#pragma endregion
#pragma region calibration parameters settings and calibration object instantiation
		
		Tinker::calibration calibration_manager;

		//calibration_manager.setup_camera_calibration_parameters(cv::Size(FLAGS_w, FLAGS_height), Size(640, 480), FLAGS_pt, 1.0, 1.0, FLAGS_n, FLAGS_d, Tinker::DETECTION, FLAGS_op, FLAGS_oe, 0, FLAGS_o);
		//calibration_manager.setup_projector_calibration_parameters(Size(1920, 1080), FLAGS_ps, Size(4,5), 80, Tinker::Pattern::ASYMMETRIC_CIRCLES_GRID, 500, 250);
		//calibration_manager.set_projector_static_image_points();
#pragma endregion


#pragma region Capture and processing loop
		int processing = 1;
		do {
			cv::Mat frame = frame_capture.read(); // Capture the frame
			if (!frame.empty()) {
				cv::imshow("raw", frame);
			}
			else {
				std::cerr << "Error: Empty frame received\n";
			}

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