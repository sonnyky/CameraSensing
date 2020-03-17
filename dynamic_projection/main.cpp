#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <state.hpp>
#include <flags.hpp>
#include "app.h"
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
			void operator()(TrackingState *  t)
			{
				cout<< " in tracking state "<< endl;
			}
			void operator()(CalibrationState * c)
			{
				cout << " in calibration state " << endl;
			}
		};

		std::variant<TrackingState * , CalibrationState * > v = capture_state.get_current_state<std::variant<TrackingState, CalibrationState>>();
		std::visit(Visitor{}, v);

#pragma endregion
#pragma region capture_initializations
		Capture capture;
		rs2::context ctx;

		ctx.set_devices_changed_callback([&](rs2::event_information& info)
		{
			capture.remove_devices(info);
			for (auto&& dev : info.get_new_devices())
			{
				capture.enable_device(dev);
			}
		});

		// Initial population of the device list
		for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
		{
			capture.enable_device(dev);
		}

		// Align to color frame
		capture.set_alignment(1);

		bool frame_available = false;

		while (!frame_available) {
			auto list_of_framesets = capture.get_depth_and_color_frameset();
			if (list_of_framesets.size() > 0) frame_available = true;
		}

		vector<rs2_intrinsics> intrinsics = capture.get_cameras_intrinsics();
		const rs2_intrinsics * camera_intrinsics = &intrinsics[0];

		cout << "intrinsics from realsense camera..." << endl;
		auto principal_point = std::make_pair(camera_intrinsics->ppx, camera_intrinsics->ppy);
		auto focal_length = std::make_pair(camera_intrinsics->fx, camera_intrinsics->fy);
		rs2_distortion model = camera_intrinsics->model;

		std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
		std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
		std::cout << "Distortion Model        : " << model << std::endl;
		std::cout << "Distortion Coefficients : [" << camera_intrinsics->coeffs[0] << "," << camera_intrinsics->coeffs[1] << "," <<
			camera_intrinsics->coeffs[2] << "," << camera_intrinsics->coeffs[3] << "," << camera_intrinsics->coeffs[4] << "]" << std::endl;


#pragma endregion Here we initialize capture parameters and enable devices

#pragma region opencv window
		int width_first = 1920;
		int height_first = 0;

		// define dimension of the second display
		int width_second = 1920;
		int height_second = 1080;

		// move the window to the second display 
		// (assuming the two displays are top aligned)
		cvNamedWindow("ProjectionWindow", WND_PROP_FULLSCREEN);
		moveWindow("ProjectionWindow", width_first, height_first);
		cvSetWindowProperty("ProjectionWindow", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

		// create target image
		Mat detectionResized = Mat(Size(width_second, height_second), CV_8UC1);
		Mat projection = Mat(Size(width_second, height_second), CV_8UC1);
#pragma endregion
#pragma region calibration parameters settings and calibration object instantiation
		
		Tinker::calibration calibration_manager;

		calibration_manager.setup_camera_calibration_parameters(cvSize(FLAGS_w, FLAGS_height), cvSize(640, 480), FLAGS_pt, 1.0, 1.0, FLAGS_n, FLAGS_d, Tinker::DETECTION, FLAGS_op, FLAGS_oe, 0, FLAGS_o);
		calibration_manager.setup_projector_calibration_parameters(cvSize(1920, 1080), FLAGS_ps, Size(4,5), 40, Tinker::Pattern::ASYMMETRIC_CIRCLES_GRID, 500, 250);
		calibration_manager.set_projector_static_image_points();
#pragma endregion
#pragma region Capture and processing loop
		int processing = 1;
		do {

			auto list_of_framesets = capture.get_depth_and_color_frameset();

			if (list_of_framesets.size() > 0) {
				int size = list_of_framesets.size();

				for (int i = 0; i < size; i++) {
					vector<Point2f> pointBuf;
					Mat view = list_of_framesets[i].color_image;
					cv::cvtColor(view, view, CV_BGR2RGB);
					
#pragma region circle detection
					Mat gray;
					cvtColor(view, gray, COLOR_BGR2GRAY);
					medianBlur(gray, gray, 5);
					
#pragma endregion

#pragma region Calibration
					calibration_manager.calibrate_camera(view);

					calibration_manager.draw_projector_pattern(view, projection);
					calibration_manager.calibrate_projector(view);
#pragma endregion
#pragma region command keys
					// Command keys
					if (waitKey(1) == 113) {
						// press 'q'
						processing = 0;
						break;
					}
					if (waitKey(1) == 99) {
						// press 'c'
						MODE = CALIBRATION_MODE;
						cout << "going to calibration mode..." << endl;
						calibration_manager.switch_to_calibration_mode();
					}
					if (waitKey(1) == 112) {
						// press 'p'
						cout << "calibrating projector..." << endl;
						calibration_manager.start_projector_calibration();
					}
#pragma endregion
					
#pragma region Display result
					

					// Get 3D coordinates of projected circles in camera coordinate system.


					// Get 3D coordinates of projected circles from board coordinates. Back projection

					// computing the instrinsics of the projector because you have 3d points (the projected circles) in world coordinates, and their respective “projection” in projector “image” plane.

					// Matrix decomposition to get K, R, T of projector

					/*
					Finally, we can start computing the extrinsics of the camera-projector (because basically we have 3d points in “world coordinates” (the board), 
					which are the projected circles, and also their projection (2d points) in the camera image and projector “image”). 
					This means you can use the standard stereo calibration routine in openCV
					*/

					// show image
					resize(view, detectionResized, CvSize(width_second, height_second));
					imshow("My Window", detectionResized);
					imshow("ProjectionWindow", projection);
#pragma endregion

				}
			}
		}while (processing == 1);

#pragma endregion Capture and processing loop
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}