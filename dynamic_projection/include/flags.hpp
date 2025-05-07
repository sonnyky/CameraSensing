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

/// @brief Message for performance counter
static const char performance_counter_message[] = "Optional. Enable per-layer performance report.";

/// @brief Message for not showing processed video
static const char no_show_processed_video[] = "Optional. Do not show processed video.";

/// @brief Message for raw output
static const char raw_output_message[] = "Optional. Output inference results as raw values.";
DEFINE_bool(h, false, help_message);

DEFINE_uint64(pattern_width, 9, help_message);
DEFINE_uint64(pattern_height, 6, help_message);
DEFINE_string(pattern_type, "chessboard", video_message);
DEFINE_uint64(minimum_frames, 10, help_message);
DEFINE_uint64(delay_between_frames, 3000, help_message);
DEFINE_uint64(s, 1, help_message);
DEFINE_string(camera_calib_filename, "camera_params.xml", video_message);
DEFINE_string(ps, "projector_params.xml", video_message);
DEFINE_bool(write_points, false, help_message);
DEFINE_bool(write_extrinsics, true, help_message);
DEFINE_uint64(zt, 8, help_message);
DEFINE_bool(su, true, help_message);


/**	
* @brief This function shows a help message
*/
static void showUsage() {
	std::cout << std::endl;
	std::cout << "dynami_projection [OPTION]" << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << std::endl;
	std::cout << "    -h                         " << help_message << std::endl;
	std::cout << "    -i \"<path>\"                " << video_message << std::endl;
	std::cout << "    -pc                        " << performance_counter_message << std::endl;
	std::cout << "    -no_show                   " << no_show_processed_video << std::endl;
	std::cout << "    -r                         " << raw_output_message << std::endl;
}

