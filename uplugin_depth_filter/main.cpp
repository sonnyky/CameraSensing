#include "common.hpp"
#include <iostream>
#include <fstream>

using namespace std;
extern "C" {

	realsense_capture* com_tinker_realsense_capture_create() {
		return new realsense_capture();
	}

	const char* com_tinker_open_device(realsense_capture* instance) {
		instance->open_device();
		return instance->get_error_message();
	}

	bool com_tinker_get_init_flag(realsense_capture* instance) {
		return instance->get_init_flag();
	}

	const char * com_tinker_get_error_message(realsense_capture* instance) {
		return instance->get_error_message();
	}

	const char * com_tinker_get_plugin_name(realsense_capture* instance) {
		return instance->get_plugin_name();
	}

	LPSTR* com_tinker_list_devices(realsense_capture* instance) {
		return instance->list_devices();
	}

	void com_tinker_remove_devices(realsense_capture* instance) {
		instance->remove_devices();
	}

	const people_position * com_tinker_get_depth(realsense_capture* instance, int &size) {
		return instance->get_depth(size);
	}

	void com_tinker_get_thresholded_image(realsense_capture* instance, unsigned char * data,  int &width, int &height) {
		instance->get_thresholded_image(data, width, height);
	}
	void com_tinker_get_color_image(realsense_capture* instance, unsigned char * data, int &width, int &height) {
		instance->get_color_image(data, width, height);
	}

	void com_tinker_stop_all_devices(realsense_capture* instance) {
		instance->close_all_devices();
	}
	void com_tinker_destroy_class(realsense_capture* instance) {
		instance->destroy_class();
	}

	const float * com_tinker_get_homography(realsense_capture* instance, float proj_tl_x, float proj_tl_y, float proj_tr_x, float proj_tr_y, float proj_bl_x, float proj_bl_y, float proj_br_x, float proj_br_y,
		float image_tl_x, float image_tl_y, float image_tr_x, float image_tr_y, float image_bl_x, float image_bl_y, float image_br_x, float image_br_y, int &size) {
		return instance->calc_homography(proj_tl_x, proj_tl_y, proj_tr_x, proj_tr_y, proj_bl_x, proj_bl_y, proj_br_x, proj_br_y,
			 image_tl_x, image_tl_y, image_tr_x, image_tr_y, image_bl_x, image_bl_y, image_br_x, image_br_y, size);
	}

	void com_tinker_setup_detection_params(realsense_capture* instance, int lowDistMin, int lowDistMax, int maxDistMin, int maxDistMax, int minBlobArea, int maxBlobArea, int erosionSize, int adjustment) {
		instance->setup_detection_params(lowDistMin, lowDistMax, maxDistMin, maxDistMax, minBlobArea, maxBlobArea, erosionSize, adjustment);
	}
}