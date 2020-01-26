#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "position_detector.h"
#include "app.h"

#include "pose_detector_flags.hpp"

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

	if (FLAGS_i.empty()) {
		throw std::logic_error("Parameter -i is not set");
	}

	if (FLAGS_m.empty()) {
		throw std::logic_error("Parameter -m is not set");
	}

	return true;
}


int main(int argc, char* argv[])
{
	
	try {
		if (!ParseAndCheckCommandLine(argc, argv)) {
			return EXIT_SUCCESS;
		}
		human_pose_estimation::pose_detector estimator(FLAGS_m, FLAGS_d, FLAGS_pc_msg);


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

		capture.set_alignment(1);

		int delay = 33;
		double inferenceTime = 0.0;

		while (1) {
			if (waitKey(1) == 113) break;
			auto list_of_color_images = capture.get_color_images();
			if (list_of_color_images.size() > 0) {
				int size = list_of_color_images.size();
				for (int i = 0; i < size; i++) {
					double t1 = static_cast<double>(cv::getTickCount());
					std::vector<human_pose_estimation::HumanPose> poses = estimator.estimate(list_of_color_images[i]);
					double t2 = static_cast<double>(cv::getTickCount());
					if (inferenceTime == 0) {
						inferenceTime = (t2 - t1) / cv::getTickFrequency() * 1000;
					}
					else {
						inferenceTime = inferenceTime * 0.95 + 0.05 * (t2 - t1) / cv::getTickFrequency() * 1000;
					}
					if (FLAGS_r) {
						for (human_pose_estimation::HumanPose const& pose : poses) {
							std::stringstream rawPose;
							rawPose << std::fixed << std::setprecision(0);
							for (auto const& keypoint : pose.keypoints) {
								rawPose << keypoint.x << "," << keypoint.y << " ";
							}
							rawPose << pose.score;
							std::cout << rawPose.str() << std::endl;
						}
					}
					imshow(to_string(i), list_of_color_images[i]);
				}
			}
		}


	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}