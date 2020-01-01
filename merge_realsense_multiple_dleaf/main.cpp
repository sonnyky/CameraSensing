#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "app.h"
#include <signal.h>

int INTERRUPT = 0;

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

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
	ofstream myfile;
	auto end = std::chrono::system_clock::now();
	std::time_t end_time = std::chrono::system_clock::to_time_t(end);

	switch (fdwCtrlType)
	{
		// Handle the CTRL-C signal. 
	case CTRL_C_EVENT:
		printf("Ctrl-C event\n\n");
		myfile.open("CTRL_C_EVENT.txt");
		myfile << "Writing this to a file.\n";
		myfile.close();
		return TRUE;

		// CTRL-CLOSE: confirm that the user wants to exit. 
	case CTRL_CLOSE_EVENT:
		printf("Ctrl-Close event\n\n");
		return TRUE;

		// Pass other signals to the next handler. 
	case CTRL_BREAK_EVENT:
		myfile.open("CTRL_BREAK_EVENT.txt");
		myfile << "Time interrupt received : \n";
		myfile << std::ctime(&end_time);
		myfile.close();

		INTERRUPT = 1;

		printf("Ctrl-Break event\n\n");
		return FALSE;

	case CTRL_LOGOFF_EVENT:
		printf("Ctrl-Logoff event\n\n");
		return FALSE;

	case CTRL_SHUTDOWN_EVENT:
		printf("Ctrl-Shutdown event\n\n");
		return FALSE;

	default:
		return FALSE;
	}
}

int main(int argc, char* argv[])
{
	if (SetConsoleCtrlHandler(CtrlHandler, TRUE))
	{

		try {
			// Instantiate new capture class that provides methods to interact with the camera
			Capture capture;

			// Create librealsense context for managing devices
			rs2::context ctx;

			// Capture parameters setup
			if (argc >= 10) {
				cout << "Detection mode. Setting up detection parameters..." << endl;
				capture.set_server_ip(argv[1]);
				capture.set_detection_params(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]), atoi(argv[9]));
				capture.read_homography_file();
			}
			else if (argc == 2) {
				cout << "Detection mode. changing IP to : " << argv[1] << endl;
				capture.set_server_ip(argv[1]);
			}
			else {
				cout << "Detection mode. Server IP is set to local machine. Setting up default parameters..." << endl;
				capture.set_default_params();
			}

			bool need_detection_window = cmdOptionExists(argv, argv + argc, "-display");
			if (need_detection_window) {
				capture.set_detection_display_param(true);
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
			while (INTERRUPT == 0) {
				capture.run();
			}
		}
		catch (std::exception& ex) {
			std::cout << ex.what() << std::endl;
		}
	}
	return EXIT_SUCCESS;
}