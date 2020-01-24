#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "app.h"

void PressEnterToContinue()
{
	std::cout << "Press ENTER to continue... " << flush;
	std::cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
}

int main(int argc, char* argv[])
{
	
	try {
		Capture capture;

		rs2::context ctx;    // Create librealsense context for managing devices

							 // Register callback for tracking which devices are currently connected

							 // Parameter setup.

		ctx.set_devices_changed_callback([&](rs2::event_information& info)
		{
			capture.remove_devices(info);
			for (auto&& dev : info.get_new_devices())
			{
				capture.enable_device(dev);
			}
		});

		// Initial population of the device list
		for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
		{
			capture.enable_device(dev);
		}
		//PressEnterToContinue();

		// Start capturing and blob detection
		capture.run();


		// Test get one color image after application ended
		auto list_of_images = capture.get_color_images();
		if (list_of_images.size() > 0) {
			for (int i = 0; i < list_of_images.size(); i++) {
				string fileName = "image" + to_string(i) + ".jpg";
				imwrite(fileName, list_of_images[i]);
			}
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return EXIT_SUCCESS;
}