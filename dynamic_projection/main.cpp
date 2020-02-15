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
		cvNamedWindow("My Window", CV_WINDOW_AUTOSIZE);
		moveWindow("My Window", width_first, height_first);
		cvSetWindowProperty("My Window", WND_PROP_AUTOSIZE, WINDOW_AUTOSIZE);

		// create target image
		Mat img = Mat(Size(width_second, height_second), CV_8UC1);
#pragma endregion

		// create circular calibration pattern
		int radius = 5;
		int distance = 30;
		int x_offset = 30; int y_offset = 30;

		vector<CvPoint> circles;
		for (int i = 0; i < 11; i++) {
			for (int j = 0; j < 4; j++) {
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
			
					// Draw circles on screen
					if (MODE == TRACKING_MODE) {
						for (int i = 0; i < circles.size(); i++) {
							circle(view, circles[i], radius, CvScalar(255, 255, 255), -1, 8, 0);
						}
					}
					// Detect circles projected by the projector
					

					// show image
					resize(view, img, CvSize(width_second, height_second));
					imshow("My Window", img);

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