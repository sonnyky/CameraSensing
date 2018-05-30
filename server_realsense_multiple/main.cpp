#undef UNICODE

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "proto/position.pb.h"

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "1101"

using namespace std;

void PressEnterToContinue()
{
	std::cout << "Press ENTER to continue... " << flush;
	std::cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
}

int __cdecl main(void)
{
	WSADATA wsaData;
	int iResult, iDataResult;
	int messagelength = 0;

	SOCKET ListenSocket = INVALID_SOCKET;
	SOCKET ClientSocket = INVALID_SOCKET;

	struct addrinfo *result = NULL;
	struct addrinfo hints;

	int iSendResult;
	char recvbuf[DEFAULT_BUFLEN];
	const char * recvdata = "data received";
	int recvbuflen = DEFAULT_BUFLEN;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Create a SOCKET for connecting to server
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 1;
	}

	// Setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);

	iResult = listen(ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	// Accept a client socket
	ClientSocket = accept(ListenSocket, NULL, NULL);
	if (ClientSocket == INVALID_SOCKET) {
		printf("accept failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	// No longer need server socket
	closesocket(ListenSocket);

	// Receive until the peer shuts down the connection
	do {
		cout << "Messagelength before recv : " << to_string(messagelength) << endl;
		iResult = recv(ClientSocket, (char *)&messagelength, sizeof(int), 0);
		if (iResult > 0) {
			printf("iResult Bytes received: %d\n", iResult);
			printf("Data received: %d\n", (int)messagelength);
		}
		else if (iResult == 0) {
			printf("Connection closing...\n");
			PressEnterToContinue();
		}
		else {
			printf("recv failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			PressEnterToContinue();
			return 1;
		}
		//// must recv again here after getting the length
		zaboom::people_position recv_positions;
		string test;
		vector<char> recv_char(messagelength);
		iDataResult = recv(ClientSocket, recv_char.data(), recv_char.size() - 1, 0);

		string rcv;
		zaboom::people_position test_elem, last_elem;

		/*for (int i = 0; i < (int)messagelength; i++) {
			vector<char> recv_char(sizeof(zaboom::people_position));
			iDataResult = recv(ClientSocket, (char *)&recv_char[0], sizeof(zaboom::people_position), 0);
			string one_element_string;
			one_element_string.append(recv_char.cbegin(), recv_char.cbegin() + sizeof(zaboom::people_position));

			zaboom::people_position test_elem;
			test_elem.ParseFromString(one_element_string);
			cout << "test element : " << test_elem.DebugString() << endl;
		}*/
		if (iDataResult > 0) {
			printf("iDataResult Bytes received: %d\n", iDataResult);
			test.append(recv_char.begin(), recv_char.end());
			last_elem.ParseFromString(test);
			cout << "number of elements in object : " << to_string( last_elem.ByteSize()) << endl;
			cout << "deserialize string : " << endl;
			cout << last_elem.DebugString() << endl;
			//rcv.append(recv_char.cbegin(), recv_char.cend());
			/*string one_element_string;

			one_element_string.append(recv_char.cbegin(), recv_char.cbegin() + sizeof(zaboom::people_position));
			cout <<"test element : " << test_elem.ParseFromString(one_element_string) << endl;
			cout << "test element : " << test_elem.DebugString() << endl;

			string last_element_string;
			last_element_string.append(recv_char.cbegin() + sizeof(zaboom::people_position) + 1, recv_char.cbegin() + sizeof(zaboom::people_position)+ sizeof(zaboom::people_position));
			cout << "last element : " << last_elem.ParseFromString(last_element_string) << endl;
			cout << "last element : " << last_elem.DebugString() << endl;
*/
			//cout << "size of one element : " << to_string(recv_char.cend()  - recv_char.cbegin())<< endl;
			//cout << "size of one element : " << sizeof(zaboom::people_position) << endl;

			//// so if we define a second variable
			//string second_test; zaboom::people_position second_elem_test;
			//second_test.append(recv_char.cbegin(), recv_char.cbegin() + 40);
			//cout << "second test : " << second_elem_test.ParseFromString(second_test) << endl;
			//cout << "second test : " << second_elem_test.DebugString()<< endl;
		}
		else if (iDataResult == 0) {
			printf("iDataResult Connection closing...\n");
			PressEnterToContinue();
		}
		else {
			printf("recv failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			PressEnterToContinue();
			return 1;
		}

	} while (iResult > 0);

	// shutdown the connection since we're done
	iResult = shutdown(ClientSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("shutdown failed with error: %d\n", WSAGetLastError());
		closesocket(ClientSocket);
		WSACleanup();
		return 1;
	}

	// cleanup
	closesocket(ClientSocket);
	WSACleanup();

	return 0;
}