#define common_api __declspec(dllexport) 

#include "recognition.h"
#include "detection.h"

extern "C" {
	common_api recognition* com_tinker_recognition_create();

	common_api const char * com_tinker_recognition_get_plugin_name(recognition* instance);
	common_api void com_tinker_recognition_destroy_class(recognition* instance);

	common_api void com_tinker_recognition_setup_camera(recognition* instance);
	common_api void com_tinker_recognition_get_color_image(recognition* instance, unsigned char * data, int &width, int &height);
	common_api void com_tinker_recognition_release_camera(recognition* instance);
	common_api void com_tinker_recognition_detect_faces(recognition* instance, unsigned char * input, unsigned char * processed, int width, int height);
}