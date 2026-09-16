#include "include/flags.hpp"

DEFINE_bool(h, false, help_message);
DEFINE_uint64(pattern_width, 9, pattern_dimensions_message);
DEFINE_uint64(pattern_height, 6, pattern_dimensions_message);
DEFINE_uint64(num_boards_before_dynamic_projector_calib, 8, nboards_before_dynamic_message);
DEFINE_uint64(num_boards_final_projector_calib, 5, nboards_final_proj_calib_message);
DEFINE_uint64(minimum_frames, 8, minimum_frames_message);
DEFINE_uint64(delay_between_frames, 1000, delay_frames_message);
DEFINE_double(minimum_board_motion_px, 15.0, board_motion_message);
DEFINE_string(calibration_output_dir, "", calibration_output_dir_message);
DEFINE_double(projector_offset_y_scale, 0.9, projector_offset_scale_message);
DEFINE_uint64(projected_circle_radius, 30, projector_circle_radius_message);
DEFINE_double(projector_smoothing_rate, 0.4, projector_smoothing_rate_message);
DEFINE_double(max_dynamic_stereo_rms, 3.0, dynamic_stereo_rms_message);
DEFINE_uint64(calibration_candidate_margin, 4, calibration_candidate_margin_message);
DEFINE_double(max_camera_per_view_rms, 2.0, max_camera_per_view_rms_message);
DEFINE_double(max_camera_rms, 1.5, max_camera_rms_message);
DEFINE_double(max_projector_per_view_rms, 3.0, max_projector_per_view_rms_message);
DEFINE_double(max_projector_rms, 2.5, max_projector_rms_message);
DEFINE_double(minimum_calibration_position_span, 0.25, minimum_calibration_position_span_message);
DEFINE_double(minimum_calibration_distance_ratio, 1.1, minimum_calibration_distance_ratio_message);
DEFINE_double(minimum_calibration_orientation_span_deg, 10.0, minimum_calibration_orientation_span_message);
