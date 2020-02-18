#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <state.hpp>
#include <flags.hpp>
#include "app.h"

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

		cout << " capture " << endl;

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

#pragma endregion Here we initialize capture parameters and enable devices

#pragma region opencv window
		int width_first = 1920;
		int height_first = 0;

		// define dimension of the second display
		int width_second = 1280;
		int height_second = 960;

		// move the window to the second display 
		// (assuming the two displays are top aligned)
		cvNamedWindow("ProjectionWindow", CV_WINDOW_AUTOSIZE);
		moveWindow("ProjectionWindow", width_first, height_first);
		cvSetWindowProperty("ProjectionWindow", WND_PROP_AUTOSIZE, WINDOW_AUTOSIZE);

		// create target image
		Mat detectionResized = Mat(Size(width_second, height_second), CV_8UC1);
		Mat projectionResized = Mat(Size(width_second, height_second), CV_8UC1);
#pragma endregion

		// create circular calibration pattern
		int radius = 25;
		int distance = 70;
		int x_offset = 30; int y_offset = 30;

		vector<CvPoint> circles;
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 5; j++) {
				CvPoint point{ x_offset + ((i % 2) * distance + (j * distance * 2)), y_offset + (i * distance )};
				circles.push_back(point);
			}
		}
#pragma region Capture and processing loop
		while (1) {
			if (waitKey(1) == 113) break;
			
			auto list_of_framesets = capture.get_depth_and_color_frameset();

			if (list_of_framesets.size() > 0) {
				int size = list_of_framesets.size();

				for (int i = 0; i < size; i++) {
					vector<Point2f> pointBuf;
					Mat view = list_of_framesets[i].color_image;
					cv::cvtColor(view, view, CV_BGR2RGB);

					Mat circlesDisplay(cvSize(view.cols, view.rows), CV_8UC3, Scalar(0));
			
					// Draw circles on screen
					if (MODE == TRACKING_MODE) {
						for (int i = 0; i < circles.size(); i++) {
							circle(circlesDisplay, circles[i], radius, CvScalar(255, 255, 255), -1, 8, 0);
						}
					}
					
					// Detect circles projected by the projector
#pragma region circle detection
					Mat gray;
					cvtColor(view, gray, COLOR_BGR2GRAY);
					medianBlur(gray, gray, 5);

					vector<Vec3f> circles;
					HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
						gray.rows / 64,  // change this value to detect circles with different distances to each other
						100, 30, 1, 30 // change the last two parameters
				   // (min_radius & max_radius) to detect larger circles
					);
#pragma endregion
#pragma region Display result
					// Draw found circles on image
					for (size_t i = 0; i < circles.size(); i++)
					{
						Vec3i c = circles[i];
						Point center = Point(c[0], c[1]);
						// circle center
						circle(view, center, 1, Scalar(0, 100, 100), 2, LINE_AA);
						// circle outline
						int radius = c[2];
						circle(view, center, radius, Scalar(255, 0, 255), 2, LINE_AA);
					}

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
					resize(circlesDisplay, projectionResized, CvSize(width_second, height_second));
					imshow("My Window", detectionResized);
					imshow("ProjectionWindow", projectionResized);
#pragma endregion
				}
			}

		}

#pragma endregion Capture and processing loop
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}