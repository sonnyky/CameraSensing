#include "common.hpp"
#include <iostream>
#include <fstream>

using namespace std;
extern "C" {

	cv_util* com_tinker_cv_util_create() {
		return new cv_util();
	}

	const char * com_tinker_cv_util_get_error_message(cv_util* instance) {
		return instance->get_error_message();
	}

	const char * com_tinker_cv_util_get_plugin_name(cv_util* instance) {
		return instance->get_plugin_name();
	}

	void com_tinker_cv_util_destroy_class(cv_util* instance) {
		instance->destroy_class();
	}

	void com_tinker_cv_util_calc_homography(cv_util* instance, UPoint * src, UPoint * dst, int length, float * data) {
		 return instance->calc_homography( src, dst, length, data);
	}
}