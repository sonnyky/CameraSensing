#define common_api __declspec(dllexport) 

#include "cv_depth.h"
#include "detection.h"

extern "C" {
	common_api cv_depth* com_tinker_cv_depth_create();

	common_api const char * com_tinker_cv_depth_get_plugin_name(cv_depth* instance);
	common_api const char * com_tinker_cv_depth_get_error_message(cv_depth* instance);

	common_api const float * com_tinker_cv_depth_calc_homography(cv_depth* instance, UPoint * src, UPoint * dst, int length);
	common_api void com_tinker_cv_depth_destroy_class(cv_depth* instance);
}