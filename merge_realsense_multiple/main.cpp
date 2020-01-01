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

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
	{
		return *itr;
	}
	return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

int main(int argc, char* argv[])
{
	
	try {
		// Instantiate new capture class that provides methods to interact with the camera
		Capture capture;

		// Create librealsense context for managing devices
		rs2::context ctx;

		// Capture parameters setup
		if (argc >= 11) {
			cout << "Detection mode. Setting up detection parameters..." << endl;
			capture.set_server_ip(argv[1]);
			capture.set_detection_params(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]), atoi(argv[9]));
			capture.set_adjustment_mode(atoi(argv[10]));
			capture.read_homography_file();
		}
		else if (argc == 3) {
			cout << "Detection mode. changing IP to : " << argv[1] << endl;
			capture.set_server_ip(argv[1]);
			capture.set_adjustment_mode(atoi(argv[2]));
		}
		else {
			cout << "Detection mode. Server IP is set to local machine. Setting up default parameters..." << endl;
			capture.set_default_params();
		}

		capture.set_normalize_flag(cmdOptionExists(argv, argv + argc, "-norm"));

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