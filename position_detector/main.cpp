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

			// Retrieve color and depth sets from realsense cameras
			
			auto list_of_framesets = capture.get_depth_and_color_frameset();

			vector<rs2_intrinsics> intrinsics = capture.get_cameras_intrinsics();

			if (list_of_framesets.size() > 0) {
				int size = list_of_framesets.size();
				for (int i = 0; i < size; i++) {
					double t1 = static_cast<double>(cv::getTickCount());

					// Get pose estimates
					std::vector<human_pose_estimation::HumanPose> poses = estimator.estimate(list_of_framesets[i].color_image);
					double t2 = static_cast<double>(cv::getTickCount());
					if (inferenceTime == 0) {
						inferenceTime = (t2 - t1) / cv::getTickFrequency() * 1000;
					}
					else {
						inferenceTime = inferenceTime * 0.95 + 0.05 * (t2 - t1) / cv::getTickFrequency() * 1000;
					}

					// Rendering to the image
					human_pose_estimation::renderHumanPose(poses, list_of_framesets[i].color_image);

					for (human_pose_estimation::HumanPose const& pose : poses) {
						// test draw on the nose node
						circle(list_of_framesets[i].color_image, cvPoint(pose.keypoints[0].x, pose.keypoints[0].y), 20, Scalar(255, 255, 255), CV_FILLED, 8, 0);

						// get the node 3d position from camera.
						float distance = capture.get_distance_at_pixel(pose.keypoints[0].x, pose.keypoints[0].y, (depth_frame) list_of_framesets[i].depth_frame);

						float point3d[3];

						const float pixel[2] = { pose.keypoints[0].x , pose.keypoints[0].y};
						const rs2_intrinsics * camera_intrinsics = &intrinsics[0];
					
						rs2_deproject_pixel_to_point(point3d, camera_intrinsics, pixel, distance);

						string log_position = to_string(point3d[0]) + ", " + to_string(point3d[1]) + ", " + to_string(point3d[2]);

						cv::putText(list_of_framesets[i].color_image, log_position, cv::Point(16, 32),
							cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255));
					}

					
					imshow(to_string(i), list_of_framesets[i].color_image);
				}
			}
		}


	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}