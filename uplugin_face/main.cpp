#include "common.hpp"
#include <iostream>
#include <fstream>

using namespace std;
extern "C" {

	recognition* com_tinker_recognition_create() {
		return new recognition();
	}

	const char * com_tinker_recognition_get_plugin_name(recognition* instance) {
		return instance->get_plugin_name();
	}

	void com_tinker_recognition_destroy_class(recognition* instance) {
		instance->destroy_class();
	}

	void com_tinker_recognition_setup_camera(recognition* instance) {
		instance->setup_camera();
	}

	void com_tinker_recognition_get_color_image(recognition* instance, unsigned char * data, int &width, int &height) {
		instance->get_color_image(data, width, height);
	}

	void com_tinker_recognition_release_camera(recognition* instance) {
		instance->release_camera();
	}
	common_api void com_tinker_recognition_detect_faces(recognition * instance, unsigned char * input, unsigned char * processed, int width, int height)
	{
		return common_api void();
	}
}