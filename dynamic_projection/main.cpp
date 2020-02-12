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

		//using state_variants = ;
		std::variant<TrackingState * , CalibrationState * > v = capture_state.get_current_state<std::variant<TrackingState, CalibrationState>>();

		std::visit(Visitor{}, v);

		
//
//
//#pragma region capture_initializations
//		Capture capture;
//		rs2::context ctx;
//
//		ctx.set_devices_changed_callback([&](rs2::event_information& info)
//		{
//			capture.remove_devices(info);
//			for (auto&& dev : info.get_new_devices())
//			{
//				capture.enable_device(dev);
//			}
//		});
//
//		// Initial population of the device list
//		for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
//		{
//			capture.enable_device(dev);
//		}
//
//		capture.set_alignment(1);
//
//		bool frame_available = false;
//
//		while (!frame_available) {
//			auto list_of_framesets = capture.get_depth_and_color_frameset();
//			if (list_of_framesets.size() > 0) frame_available = true;
//		}
//
//		vector<rs2_intrinsics> intrinsics = capture.get_cameras_intrinsics();
//		const rs2_intrinsics * camera_intrinsics = &intrinsics[0];
//
//#pragma endregion Here we initialize capture parameters and enable devices
//
//#pragma region Capture and processing loop
//		while (1) {
//			if (waitKey(1) == 113) break;
//			
//			auto list_of_framesets = capture.get_depth_and_color_frameset();
//
//
//
//		}
//
//#pragma endregion Capture and processing loop
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}