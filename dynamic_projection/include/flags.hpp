// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <gflags/gflags.h>
#include <iostream>

/// @brief Message for help argument
static const char help_message[] = "Print a usage message.";

/// @brief Message for calibration pattern dimensions
static const char pattern_dimensions_message[] = "Number of inner chessboard corners along the corresponding width or height.";

static const char nboards_before_dynamic_message[] = "Number of retained static-projector views required before dynamic calibration; extra candidates are collected separately.";

static const char nboards_final_proj_calib_message[] = "Minimum accepted dynamic-projector samples for completion, also subject to the stereo RMS limit.";

static const char minimum_frames_message[] = "Number of retained camera calibration views; extra candidates are collected separately.";

static const char delay_frames_message[] = "Minimum interval in milliseconds between accepted calibration samples after the first sample in each phase.";
static const char board_motion_message[] = "Minimum chessboard corner RMS movement in pixels required to accept another calibration sample.";

static const char calibration_output_dir_message[] = "Directory for calibration result files. Empty uses the executable directory.";
static const char projector_offset_scale_message[] = "Scale factor for the Y-axis offset used during dynamic projector calibration.";
static const char projector_circle_radius_message[] = "Radius in pixels for the projected circle pattern.";
static const char projector_smoothing_rate_message[] = "Smoothing rate for dynamic projector pose and image-point updates.";
static const char dynamic_stereo_rms_message[] = "Maximum stereo RMS error allowed before leaving dynamic calibration.";
static const char calibration_candidate_margin_message[] = "Extra accepted views collected before camera or static-projector view selection.";
static const char max_camera_per_view_rms_message[] = "Maximum camera per-view RMS eligible for final view selection.";
static const char max_camera_rms_message[] = "Maximum aggregate camera RMS allowed for calibration completion.";
static const char max_projector_per_view_rms_message[] = "Maximum projector per-view RMS eligible for final view selection.";
static const char max_projector_rms_message[] = "Maximum aggregate projector RMS allowed for calibration completion.";
static const char minimum_calibration_position_span_message[] = "Minimum normalized centroid span at view selection: chessboard corners for camera, projected circles for static projector.";
static const char minimum_calibration_distance_ratio_message[] = "Minimum far-to-near absolute board-origin Z translation ratio at view selection, before the final camera refit.";
static const char minimum_calibration_orientation_span_message[] = "Minimum board-normal angular span in degrees at view selection, before the final camera refit.";




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
DECLARE_uint64(calibration_candidate_margin);
DECLARE_double(max_camera_per_view_rms);
DECLARE_double(max_camera_rms);
DECLARE_double(max_projector_per_view_rms);
DECLARE_double(max_projector_rms);
DECLARE_double(minimum_calibration_position_span);
DECLARE_double(minimum_calibration_distance_ratio);
DECLARE_double(minimum_calibration_orientation_span_deg);


/**	
* @brief This function shows a help message
*/
static void showUsage() {
	gflags::ShowUsageWithFlags("dynamic_projection");
}

