#include "include/flags.hpp"

DEFINE_bool(h, false, help_message);
DEFINE_uint64(pattern_width, 9, help_message);
DEFINE_uint64(pattern_height, 6, help_message);
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
