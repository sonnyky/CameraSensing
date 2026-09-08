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

static const char nboards_before_dynamic_message[] = "Required. Specifies the number of calibration images necessary before moving to dynamic projector calibration.";

static const char nboards_final_proj_calib_message[] = "Required. Specifies the number of calibration images necessary to finish projector calibration.";

static const char minimum_frames_message[] = "Required. Specifies the number of calibration images necessary for camera calibration.";

static const char delay_frames_message[] = "Required. Specifies the time in ms to delay frame capture during camera and projector calibration.";
static const char board_motion_message[] = "Minimum chessboard corner RMS movement in pixels required to accept another calibration sample.";

static const char calibration_output_dir_message[] = "Directory for calibration result files. Empty uses the executable directory.";
static const char projector_offset_scale_message[] = "Scale factor for the Y-axis offset used during dynamic projector calibration.";
static const char projector_circle_radius_message[] = "Radius in pixels for the projected circle pattern.";
static const char projector_smoothing_rate_message[] = "Smoothing rate for dynamic projector pose and image-point updates.";
static const char dynamic_stereo_rms_message[] = "Maximum stereo RMS error allowed before leaving dynamic calibration.";




DECLARE_bool(h);
DECLARE_uint64(pattern_width);
DECLARE_uint64(pattern_height);
DECLARE_uint64(num_boards_before_dynamic_projector_calib);
DECLARE_uint64(num_boards_final_projector_calib);
DECLARE_uint64(minimum_frames);
DECLARE_uint64(delay_between_frames);
DECLARE_double(minimum_board_motion_px);
DECLARE_string(calibration_output_dir);
DECLARE_double(projector_offset_y_scale);
DECLARE_uint64(projected_circle_radius);
DECLARE_double(projector_smoothing_rate);
DECLARE_double(max_dynamic_stereo_rms);


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

}

