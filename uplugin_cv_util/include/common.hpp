#define common_api __declspec(dllexport) 

#include "cv_util.h"
#include "homography.h"

extern "C" {
	common_api cv_util* com_tinker_cv_util_create();

	common_api const char * com_tinker_cv_util_get_plugin_name(cv_util* instance);
	common_api const char * com_tinker_cv_util_get_error_message(cv_util* instance);

	common_api const float * com_tinker_cv_util_calc_homography(cv_util* instance, UPoint * src, UPoint * dst, int length);
	common_api void com_tinker_cv_util_destroy_class(cv_util* instance);
}