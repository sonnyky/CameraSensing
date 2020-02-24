// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <gflags/gflags.h>
#include <iostream>

/// @brief Message for help argument
static const char help_message[] = "Print a usage message.";

/// @brief Message for video argument
static const char video_message[] = "Required. Path to a video. Default value is \"cam\" to work with camera.";

/// @brief Message for model argument
static const char human_pose_estimation_model_message[] = "Required. Path to the Human Pose Estimation model (.xml) file.";

/// @brief Message for assigning Human Pose Estimation inference to device
static const char target_device_message[] = "Optional. Specify the target device for Human Pose Estimation "\
"(the list of available devices is shown below). Default value is CPU. " \
"Use \"-d HETERO:<comma-separated_devices_list>\" format to specify HETERO plugin. " \
"The application looks for a suitable plugin for the specified device.";

/// @brief Message for performance counter
static const char performance_counter_message[] = "Optional. Enable per-layer performance report.";

/// @brief Message for not showing processed video
static const char no_show_processed_video[] = "Optional. Do not show processed video.";

/// @brief Message for raw output
static const char raw_output_message[] = "Optional. Output inference results as raw values.";

DEFINE_uint64(w, 7, help_message);
DEFINE_uint64(h, 4, help_message);
DEFINE_string(pt, "chessboard", video_message);
DEFINE_uint64(n, 10, help_message);
DEFINE_uint64(d, 1000, help_message);
DEFINE_uint64(s, 1, help_message);
DEFINE_string(o, "camera_params.xml", video_message);
DEFINE_bool(op, false, help_message);
DEFINE_bool(oe, true, help_message);
DEFINE_uint64(zt, 8, help_message);
DEFINE_bool(su, true, help_message);


/**	
* @brief This function shows a help message
*/
static void showUsage() {
	std::cout << std::endl;
	std::cout << "human_pose_estimation_demo [OPTION]" << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << std::endl;
	std::cout << "    -h                         " << help_message << std::endl;
	std::cout << "    -i \"<path>\"                " << video_message << std::endl;
	std::cout << "    -m \"<path>\"                " << human_pose_estimation_model_message << std::endl;
	std::cout << "    -d \"<device>\"              " << target_device_message << std::endl;
	std::cout << "    -pc                        " << performance_counter_message << std::endl;
	std::cout << "    -no_show                   " << no_show_processed_video << std::endl;
	std::cout << "    -r                         " << raw_output_message << std::endl;
}

