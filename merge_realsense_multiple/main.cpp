#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "app.h"

void PressEnterToContinue()
{
	std::cout << "Press ENTER to continue... " << flush;
	std::cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
}

vector<string> parse(string input, string delimiter) {
	size_t pos = 0;
	std::string token;
	vector<string> result;
	while ((pos = input.find(delimiter)) != std::string::npos) {
		token = input.substr(0, pos);
		//std::cout << token << std::endl;
		input.erase(0, pos + delimiter.length());
		result.push_back(token);
	}
	result.push_back(input);
	return result;
}

int main(int argc, char* argv[])
{
	
	try {
		// Instantiate new capture class that provides methods to interact with the camera
		Capture capture;

		// Create librealsense context for managing devices
		rs2::context ctx;

		// Capture parameters setup
		if (argc == 3) {
			cout << "Setting up parameters" << endl;
			capture.set_detection_params(atoi(argv[1]), atoi(argv[2]));
		}
		else {
			capture.set_default_params();
		}

		// Register callback for tracking which devices are currently connected
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

		// prepare socket
		capture.setup_socket();

		// Start capturing and blob detection
		capture.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return EXIT_SUCCESS;
}