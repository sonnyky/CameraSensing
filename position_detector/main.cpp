#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "plane_detection.h"
#include "pose_detector.hpp"
#include "render_human_pose.hpp"
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

		capture.set_alignment(1);

		bool frame_available = false;

		while (!frame_available) {
			auto list_of_framesets = capture.get_depth_and_color_frameset();
			if (list_of_framesets.size() > 0) frame_available = true;
		}

		vector<rs2_intrinsics> intrinsics = capture.get_cameras_intrinsics();
		const rs2_intrinsics * camera_intrinsics = &intrinsics[0];

#pragma endregion Here we initialize capture parameters and enable devices

#pragma region plane_solver

		PlaneSolver plane_solver;
		PlaneSolver::plane_coefficient plane = { 0, 0, 0, 0 };

#pragma endregion Initialization of PCL based plane solver object

		int delay = 33;
		double inferenceTime = 0.0;

		const cv::Point2f absentKeypoint(-1.0f, -1.0f);
		bool plane_found = false;

		while (1) {
			if (waitKey(1) == 113) break;
			
			// small d
			if (waitKey(1) == 100) {
				plane_solver.allow_data_gathering();
			}

			vector<PlaneSolver::position> detected_feet_positions;
			auto list_of_framesets = capture.get_depth_and_color_frameset();

			if (list_of_framesets.size() > 0) {
				int size = list_of_framesets.size();
				for (int i = 0; i < size; i++) {
					double t1 = static_cast<double>(cv::getTickCount());

					// Get pose estimates
					std::vector<human_pose_estimation::HumanPose> poses = estimator.estimate(list_of_framesets[i].color_image);

					if (poses.size() == 0) continue;

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

						if (pose.keypoints.size() == 0 || pose.keypoints[10] == absentKeypoint || pose.keypoints[13] == absentKeypoint) continue;

						circle(list_of_framesets[i].color_image, cvPoint(pose.keypoints[10].x, pose.keypoints[10].y), 20, Scalar(255, 255, 255), CV_FILLED, 8, 0);
						circle(list_of_framesets[i].color_image, cvPoint(pose.keypoints[13].x, pose.keypoints[13].y), 20, Scalar(255, 255, 255), CV_FILLED, 8, 0);

						// get the node 3d position from camera.
						float distance_r = capture.get_distance_at_pixel(pose.keypoints[10].x, pose.keypoints[10].y, (depth_frame) list_of_framesets[i].depth_frame);
						float distance_l = capture.get_distance_at_pixel(pose.keypoints[13].x, pose.keypoints[13].y, (depth_frame) list_of_framesets[i].depth_frame);
					
						if (distance_r == 0 || distance_l == 0) continue;

						float point3d_r[3];
						float point3d_l[3];


						const float pixel_r[2] = { pose.keypoints[10].x , pose.keypoints[10].y};
						const float pixel_l[2] = { pose.keypoints[13].x , pose.keypoints[13].y };
					
						rs2_deproject_pixel_to_point(point3d_r, camera_intrinsics, pixel_r, distance_r);
						rs2_deproject_pixel_to_point(point3d_l, camera_intrinsics, pixel_l, distance_l);

						cout << to_string(point3d_r[0]) + ", " + to_string(point3d_r[1]) + ", " + to_string(point3d_r[2]) << endl;
						cout << to_string(point3d_l[0]) + ", " + to_string(point3d_l[1]) + ", " + to_string(point3d_l[2]) << endl;

						if (plane_solver.is_data_gathering_allowed()) {
							plane_solver.obtain_positions_data(point3d_r[0], point3d_r[1], point3d_r[2]);
						}

						string log_position = to_string(point3d_r[0]) + ", " + to_string(point3d_r[1]) + ", " + to_string(point3d_r[2]);

						cv::putText(list_of_framesets[i].color_image, log_position, cv::Point(16, 32),
							cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255));

						PlaneSolver::position right = { point3d_r[0] , point3d_r[1] , point3d_r[2] };
						PlaneSolver::position left = { point3d_l[0] , point3d_l[1] , point3d_l[2] };

						detected_feet_positions.push_back(right);
						detected_feet_positions.push_back(left);

					}
					if (plane_found) {
						vector<PlaneSolver::position> projected_points = plane_solver.project_positions_on_plane(plane, detected_feet_positions);
					}
					
					imshow(to_string(i), list_of_framesets[i].color_image);
				}
			}

			// small s
			if (waitKey(1) == 115) {
				plane = plane_solver.solve();
				if (plane.a == 0 && plane.b == 0 && plane.c == 0 && plane.d == 0) {
					// do nothing
				}
				else {
					plane_found = true;
					// Send data to receiving clients
				}
			}
		}


	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}