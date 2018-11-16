#include "server.h"

#define DEFAULT_PORT "1101"

#define ADDRESS "127.0.0.1"
#define PORT 7777

using namespace std;

int __cdecl main(int argc, char* argv[])
{
	Server server;

#pragma region mode

	if (argc == 9) {
			cout << "entering calibration mode..." << endl;
			server.DetermineMergedImageDimensions();
			server.SetMode(1);
			server.setReferencePoints(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]));
	}
	else if (argc == 1) {
		cout << "entering detection mode. all detection params set to default. Unity client machine is set to localhost" << endl;
		server.SetHomography();
	}
	else if (argc == 2) {
		cout << "entering detection mode. Sending to host : " << argv[1] << endl;
		server.setup_osc_out_ip(argv[1]);
		server.SetHomography();
	}

#pragma endregion check calib or detection mode

#pragma region networking
	
	server.setup_socket();

	server.process_data();

	server.close_socket();
	server.cleanup();
#pragma endregion cleanup and discard socket

	std::cout << "Closing application..." << endl;
	server.PressEnterToContinue();
	return 0;
}