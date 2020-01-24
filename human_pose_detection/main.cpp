#include "pose_detector.hpp"

#include "pose_detector_flags.hpp"


#include "render_human_pose.hpp"


using namespace human_pose_estimation;


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

		cv::VideoCapture cap;
		cap.open(0);

		int delay = 33;
		double inferenceTime = 0.0;
		cv::Mat image;
		if (!cap.read(image)) {
			throw std::logic_error("Failed to get frame from cv::VideoCapture");
		}

		estimator.estimate(image);  // Do not measure network reshape, if it happened

		std::cout << "To close the application, press 'CTRL+C' here";
		if (!FLAGS_no_show) {
			std::cout << " or switch to the output window and press ESC key" << std::endl;
			std::cout << "To pause execution, switch to the output window and press 'p' key" << std::endl;
		}
		std::cout << std::endl;

		do {
			double t1 = static_cast<double>(cv::getTickCount());
			std::vector<HumanPose> poses = estimator.estimate(image);
			double t2 = static_cast<double>(cv::getTickCount());
			if (inferenceTime == 0) {
				inferenceTime = (t2 - t1) / cv::getTickFrequency() * 1000;
			}
			else {
				inferenceTime = inferenceTime * 0.95 + 0.05 * (t2 - t1) / cv::getTickFrequency() * 1000;
			}
			if (FLAGS_r) {
				for (HumanPose const& pose : poses) {
					std::stringstream rawPose;
					rawPose << std::fixed << std::setprecision(0);
					for (auto const& keypoint : pose.keypoints) {
						rawPose << keypoint.x << "," << keypoint.y << " ";
					}
					rawPose << pose.score;
					std::cout << rawPose.str() << std::endl;
				}
			}

			if (FLAGS_no_show) {
				continue;
			}

			renderHumanPose(poses, image);

			cv::Mat fpsPane(35, 155, CV_8UC3);
			fpsPane.setTo(cv::Scalar(153, 119, 76));
			cv::Mat srcRegion = image(cv::Rect(8, 8, fpsPane.cols, fpsPane.rows));
			cv::addWeighted(srcRegion, 0.4, fpsPane, 0.6, 0, srcRegion);
			std::stringstream fpsSs;
			fpsSs << "FPS: " << int(1000.0f / inferenceTime * 100) / 100.0f;
			cv::putText(image, fpsSs.str(), cv::Point(16, 32),
				cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255));
			cv::imshow("ICV Human Pose Estimation", image);

			int key = cv::waitKey(delay) & 255;
			if (key == 'p') {
				delay = (delay == 0) ? 33 : 0;
			}
			else if (key == 27) {
				break;
			}
		} while (cap.read(image));

	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (...) {
		std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}