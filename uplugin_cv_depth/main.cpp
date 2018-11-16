#include "common.hpp"
#include <iostream>
#include <fstream>

using namespace std;
extern "C" {

	cv_depth* com_tinker_cv_depth_create() {
		return new cv_depth();
	}

	const char * com_tinker_cv_depth_get_error_message(cv_depth* instance) {
		return instance->get_error_message();
	}

	const char * com_tinker_cv_depth_get_plugin_name(cv_depth* instance) {
		return instance->get_plugin_name();
	}

	void com_tinker_cv_depth_destroy_class(cv_depth* instance) {
		instance->destroy_class();
	}

	const float * com_tinker_cv_depth_calc_homography(cv_depth* instance, UPoint * src, UPoint * dst, int length) {
		 return instance->calc_homography( src, dst, length);
	}
}