#pragma once

//#include "Windows/WindowsSystemIncludes.h"

#include "AllowWindowsPlatformTypes.h"
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <vector>
#include <WinSock2.h>

#include <clocale>
#include <locale>
#include <string>
#include <vector>

#include "HideWindowsPlatformTypes.h"

#define CONNECTION_WAIT_TIME 2000



typedef std::chrono::high_resolution_clock Clock;

typedef unsigned char byte;

struct SSock_Quaternion {
	float x, y, z, w;				// x,y,z,w
};

class SerialReceiver {

public:

	SerialReceiver() :
		hSerial(NULL),
		connected(false)
	{


	}

	~SerialReceiver() {

	}

public:

	std::string w2s(const std::wstring &var)
	{
		static std::locale loc("");
		auto &facet = std::use_facet<std::codecvt<wchar_t, char, std::mbstate_t>>(loc);
		return std::wstring_convert<std::remove_reference<decltype(facet)>::type, wchar_t>(&facet).to_bytes(var);
	}

	std::wstring s2w(const std::string &var)
	{
		static std::locale loc("");
		auto &facet = std::use_facet<std::codecvt<wchar_t, char, std::mbstate_t>>(loc);
		return std::wstring_convert<std::remove_reference<decltype(facet)>::type, wchar_t>(&facet).from_bytes(var);
	}

	//bool openSerialConnection(LPCWSTR port)
	bool openSerialConnection(std::string port)
	{
		connected = false;

		portName = s2w(port);

		//Try to connect to the given port throuh CreateFile
		hSerial = CreateFile(portName.c_str(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);

		//Check if the connection was successfull
		if (hSerial == INVALID_HANDLE_VALUE)
		{

			closeSerialConnection();

			//If not success full display an Error
			if (GetLastError() == ERROR_FILE_NOT_FOUND) {

				//Print Error if neccessary
				printf("ERROR: Handle was not attached. Reason: %ls not available.\n", portName.c_str());

			}
			else
			{
				printf("ERROR!!!");
			}
		}
		else
		{

			//If connected we try to set the comm parameters
			DCB dcbSerialParams = { 0 };

			//Try to get the current
			if (!GetCommState(hSerial, &dcbSerialParams))
			{
				//If impossible, show an error
				printf("failed to get current serial parameters!");
			}
			else
			{
				//Define serial connection parameters for the arduino board
				//dcbSerialParams.BaudRate = CBR_115200;
				dcbSerialParams.BaudRate = CBR_57600;
				dcbSerialParams.ByteSize = 8;
				dcbSerialParams.StopBits = ONESTOPBIT;
				dcbSerialParams.Parity = NOPARITY;
				//Setting the DTR to Control_Enable ensures that the Arduino is properly
				//reset upon establishing a connection
				dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

				//Set the parameters and check for their proper application
				if (!SetCommState(hSerial, &dcbSerialParams))
				{
					printf("ALERT: Could not set Serial Port parameters");
					connected = false;
				}
				else
				{
					//If everything went fine we're connected
					connected = true;
					//Flush any remaining characters in the buffers 
					PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
					//We wait 2s as the arduino board will be reseting
					Sleep(CONNECTION_WAIT_TIME);

					printf("Serial connection on port %ls established\n", portName.c_str());

					// flush serial pipeline
					ClearCommError(hSerial, &errors, &status);
					if (status.cbInQue > 0)
					{
						int byteAvailable = status.cbInQue;
						DWORD byteflushed = status.cbInQue;
						char *temp = new char[byteAvailable];
						ReadFile(hSerial, temp, byteAvailable, &byteflushed, NULL);
						delete temp;
					}


					t1 = Clock::now();

					return true;

				}
			}


		}

		return false;

	}

	inline u_short ByteSwap(u_short in)
	{
		u_short out;
		char *indata = (char *)&in;
		char *outdata = (char *)&out;
		outdata[0] = indata[1];
		outdata[1] = indata[0];
		return out;
	}

	SSock_Quaternion packedCharToQuaternion(const char* c) {

		SSock_Quaternion q;
		memcpy(&q, c, sizeof(byte) * 16);

		return q;
	}

	bool closeSerialConnection() {

		Sleep(100);

		connected = false;
		std::cout << "Closing connection..." << std::endl;
		int success = (int)CloseHandle(hSerial);
		if (success == 0)
			return false;
		return true;	// close serial connection
	}

	int ReadData(char *buffer, const unsigned int nbChar)
	{
		//Number of bytes we'll have read
		DWORD bytesRead;
		//Number of bytes we'll really ask to read
		unsigned int toRead;

		//Use the ClearCommError function to get status info on the Serial port
		ClearCommError(hSerial, &errors, &status);

		//Check if there is something to read
		if (status.cbInQue>0)
		{
			//If there is we check if there is enough data to read the required number
			//of characters, if not we'll read only the available characters to prevent
			//locking of the application.

			if (status.cbInQue >= nbChar)
			{
				toRead = nbChar;
			}
			else // not enough for one package, return
				return 0;

			//Try to read the require number of chars, and return the number of read bytes on success
			if (ReadFile(hSerial, buffer, toRead, &bytesRead, NULL))
			{
				return bytesRead;
			}

		}

		//If nothing has been read, or that an error was detected return 0
		return 0;

	}

	void computeFPS() {

		if (readResult > 0) {
			numPackages++;
		}

		t2 = Clock::now();
		timediff = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

		if (timediff > 1000000000) {
			t1 = Clock::now();
			//std::cout << "Incoming head tracking frequency : " << numPackages << " Hz" << std::endl;
			numPackages = 0;
		}

	}

	void Update() {

		readResult = ReadData(incomingData, dataLength);

		computeFPS();

		if (readResult > 0) {

			//printf("Bytes read: (0 means no data available) %i\n", readResult);

			SSock_Quaternion q = packedCharToQuaternion(incomingData);
			//printf("Sample : [%.2f,%.2f,%.2f,%.2f]\n", q.x, q.y, q.z, q.w);
			incomingSamples.push_back(q);

		}

	}

	bool isConnected() { return connected; }

	bool getNextSample(SSock_Quaternion &q)
	{
		if (incomingSamples.size() > 0) {
			q = incomingSamples[0];
			incomingSamples.erase(incomingSamples.begin());
			return true;
		}

		return false;
	}

private:

	std::wstring portName;
	HANDLE hSerial = NULL;
	COMSTAT status;	//Get various information about the connection
	DWORD errors;	//Keep track of last error
	bool connected = false;

	std::vector<char> incomingDataCollection;
	std::vector<SSock_Quaternion> incomingSamples;

	char incomingData[16] = ""; // don't forget to pre-allocate memory
	int dataLength = 16;
	int readResult = 0;

	// benchmark
	int numPackages = 0;
	std::chrono::steady_clock::time_point t1;
	std::chrono::steady_clock::time_point t2;
	long long timediff = 0;

};
