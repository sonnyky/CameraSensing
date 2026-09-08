// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <gflags/gflags.h>
#include <iostream>

/// @brief Message for help argument
static const char help_message[] = "Print a usage message.";

/// @brief Message for calibration pattern dimensions
static const char pattern_dimensions_message[] = "Required. Use pattern_width and pattern_height to specify calibration pattern dimensions.";

/// @brief Message for pattern type
static const char pattern_type_message[] = "Required. Specifies the calibration pattern type, defaults to chessboard.";

static const char nboards_before_dynamic_message[] = "Required. Specifies the number of calibration images necessary before moving to dynamic projector calibration.";

static const char nboards_final_proj_calib_message[] = "Required. Specifies the number of calibration images necessary to finish projector calibration.";

static const char minimum_frames_message[] = "Required. Specifies the number of calibration images necessary for camera calibration.";

static const char delay_frames_message[] = "Required. Specifies the time in ms to delay frame capture during camera and projector calibration.";
static const char board_motion_message[] = "Minimum chessboard corner RMS movement in pixels required to accept another calibration sample.";

static const char camera_filename_message[] = "Required. Specifies the filename to save camera parameters after calibration.";

static const char projector_filename_message[] = "Required. Specifies the filename to save projector intrinsics parameters after calibration.";
static const char projector_offset_scale_message[] = "Scale factor for the Y-axis offset used during dynamic projector calibration.";
static const char projector_circle_radius_message[] = "Radius in pixels for the projected circle pattern.";
static const char projector_smoothing_rate_message[] = "Smoothing rate for dynamic projector pose and image-point updates.";




DECLARE_bool(h);
DECLARE_uint64(pattern_width);
DECLARE_uint64(pattern_height);
DECLARE_string(pattern_type);
DECLARE_uint64(num_boards_before_dynamic_projector_calib);
DECLARE_uint64(num_boards_final_projector_calib);
DECLARE_uint64(minimum_frames);
DECLARE_uint64(delay_between_frames);
DECLARE_double(minimum_board_motion_px);
DECLARE_string(camera_filename);
DECLARE_string(projector_filename);
DECLARE_double(projector_offset_y_scale);
DECLARE_uint64(projected_circle_radius);
DECLARE_double(projector_smoothing_rate);
DECLARE_bool(write_points);
DECLARE_bool(write_extrinsics);


/**	
* @brief This function shows a help message
*/
static void showUsage() {
	std::cout << std::endl;
	std::cout << "dynami_projection [OPTION]" << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << std::endl;
	std::cout << "    -h								" << help_message << std::endl;
	std::cout << "    -pattern_width, -pattern_height   " << pattern_dimensions_message << std::endl;
	std::cout << "    -pattern_type								" << pattern_type_message << std::endl;

}

