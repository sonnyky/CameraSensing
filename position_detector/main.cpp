#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "position_detector.h"

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

int main(int argc, char* argv[])
{
	
	try {
		
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}