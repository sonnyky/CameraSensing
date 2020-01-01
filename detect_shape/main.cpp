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
		if (argc == 6) {
			cout << "Setting up parameters" << endl;
			capture.set_detection_params(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
		}
		else {
			cout << "Expecting four parameters. Setting default parameters" << endl;
			capture.set_default_params();
		}

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

		// Start capturing and blob detection
		capture.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return EXIT_SUCCESS;
}