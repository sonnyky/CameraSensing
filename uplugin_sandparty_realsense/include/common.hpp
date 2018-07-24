#define common_api __declspec(dllexport) 

#include "realsense_capture.h"

extern "C" {
	common_api realsense_capture* com_tinker_realsense_capture_create();
	common_api const char* com_tinker_open_device(realsense_capture* instance);

	common_api bool com_tinker_get_init_flag(realsense_capture* instance);
	common_api const char * com_tinker_get_plugin_name(realsense_capture* instance);
	common_api const char * com_tinker_get_error_message(realsense_capture* instance);
	common_api LPSTR* com_tinker_list_devices(realsense_capture* instance);

	common_api const uint16_t * com_tinker_get_sandbox_depth(realsense_capture* instance);
	common_api void com_tinker_get_color_image(realsense_capture* instance, unsigned char * data, int &width, int &height);

	common_api void com_tinker_stop_all_devices(realsense_capture* instance);
	common_api void com_tinker_remove_devices(realsense_capture* instance);
	common_api void com_tinker_destroy_class(realsense_capture* instance);
	common_api void com_tinker_setup_processing_params(realsense_capture* instance);
	common_api const float * com_tinker_get_homography(realsense_capture* instance, float proj_tl_x, float proj_tl_y, float proj_tr_x, float proj_tr_y, float proj_bl_x, float proj_bl_y, float proj_br_x, float proj_br_y,
		float image_tl_x, float image_tl_y, float image_tr_x, float image_tr_y, float image_bl_x, float image_bl_y, float image_br_x, float image_br_y, int &size);
}