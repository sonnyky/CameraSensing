#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "app.h"

int main(int argc, char* argv[])
{
	
	try {
		Capture capture;

		if (argc > 0) {
			for (int i = 1; i < argc; i++) {
				capture.ReadCloudFiles(argv[i]);
			}
			capture.SetCloudFromFiles();
		}

		capture.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}