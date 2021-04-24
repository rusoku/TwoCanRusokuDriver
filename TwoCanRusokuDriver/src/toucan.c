
// Copyright(C) 2018 by Steven Adler
// Copyright(C) 2021 Gediminas Simanskis, gediminas@rusoku.com
//
// This file is part of TwoCan, a plugin for OpenCPN.
//
// TwoCan is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TwoCan is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TwoCan. If not, see <https://www.gnu.org/licenses/>.
//
// NMEA2000® is a registered Trademark of the National Marine Electronics Association

#include "..\inc\toucan.h"
#include "..\common\inc\twocanerror.h"


// Separate thread to read data from the CAN device
HANDLE threadHandle;

// The thread id.
DWORD threadId;

// Event signalled when valid CAN Frame is received
HANDLE frameReceivedEvent;

// Signal that the thread has terminated
HANDLE threadFinishedEvent;

// Mutex used to synchronize access to the CAN Frame buffer
HANDLE frameReceivedMutex;

// Pointer to the caller's CAN Frame buffer
byte *canFramePtr;

// Variable to indicate RX thread state
BOOL	isRunning = FALSE;
UINT8	RxFrame[64];
ULONG   RxFrameTransferd;

// CANAL variables
long status;
long handle;

BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved) {
	switch (fdwReason) {
	case DLL_PROCESS_ATTACH:
		DebugPrintf(L"TouCAN DLL Process Attach\n");
		break;
	case DLL_THREAD_ATTACH:
		DebugPrintf(L"TouCAN DLL Thread Attach\n");
		break;
	case DLL_THREAD_DETACH:
		DebugPrintf(L"TouCAN DLL Thread Detach\n");
		break;
	case DLL_PROCESS_DETACH:
		DebugPrintf(L"TouCAN DLL Process Detach\n");
		break;
	}
	// As nothing to do, just return TRUE
	return TRUE;
}

//
// Drivername,
// returns the name of this driver
// BUG BUG could retrieve this value from the registry or from the CANAL library
//

DllExport char* DriverName(void) {
	DebugPrintf(L"TouCAN DriverName\n");
	return (char*)L"TouCAN Marine";
}

//
// Version
// return an arbitary version number for this driver
//

DllExport char* DriverVersion(void) {
	DebugPrintf(L"TouCAN DriverVersion\n");
	return (char*)L"1.1";
}

//
// Manufacturer
// return the name of this driver's hardware manufacturer
//

DllExport char* ManufacturerName(void) {
	DebugPrintf(L"TouCAN ManufacturerName\n");
	return (char*)L"Rusoku";
}

//
// Open. Connect to the adapter and get ready to start reading
// returns TWOCAN_RESULT_SUCCESS if events, mutexes and Toucan adapter configured correctly
//

DllExport int OpenAdapter(void) {
	// Create an event that is used to notify the caller of a received frame
	frameReceivedEvent = CreateEvent(NULL, FALSE, FALSE, CONST_DATARX_EVENT);

	if (frameReceivedEvent == NULL)
	{
		// Fatal Error
		DebugPrintf(L"Create FrameReceivedEvent failed (%d)\n", GetLastError());
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_CREATE_FRAME_RECEIVED_EVENT);
	}

	// Create an event that is used to notify the close method that the thread has ended
	threadFinishedEvent = CreateEvent(NULL, FALSE, FALSE, CONST_EVENT_THREAD_ENDED);

	if (threadFinishedEvent == NULL)
	{
		// Fatal Error
		DebugPrintf(L"Create ThreadFinished Event failed (%d)\n", GetLastError());
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_CREATE_THREAD_COMPLETE_EVENT);
	}

	// Open the mutex that is used to synchronize access to the Can Frame buffer
	// Initial state set to true, meaning we "own" the initial state of the mutex
	frameReceivedMutex = OpenMutex(SYNCHRONIZE, TRUE, CONST_MUTEX_NAME);

	if (frameReceivedMutex == NULL)
	{
		// Fatal error
		DebugPrintf(L"Open Mutex failed (%d)\n", GetLastError());
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_CREATE_FRAME_RECEIVED_MUTEX);
	}

	Toucan_winusb_deinit(&deviceData);

	Sleep(100);

	hr = Toucan_winusb_init(&deviceData, &noDevice);

	if (FAILED(hr)) {
		DebugPrintf(L"Toucan_winusb_init failed\n");
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_ADAPTER_NOT_FOUND);
	}

	if(TouCAN_init() == FALSE )
	{
		DebugPrintf(L"Toucan_init failed\n");
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_ADAPTER_NOT_FOUND);
	}

	if (TouCAN_start() == FALSE)
	{
		DebugPrintf(L"Toucan_start failed\n");
		return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_ADAPTER_NOT_FOUND);

	}

	return TWOCAN_RESULT_SUCCESS;
}

DllExport int CloseAdapter(void) {
	DebugPrintf(L"TouCAN CloseAdapter\n");

	// Terminate the read thread
	isRunning = FALSE;

	// Wait for the thread to exit
	int waitResult;
	waitResult = WaitForSingleObject(threadFinishedEvent, 1000);
	if (waitResult == WAIT_TIMEOUT) {
		DebugPrintf(L"Wait for threadFinishedEVent timed out");
	}
	if (waitResult == WAIT_ABANDONED) {
		DebugPrintf(L"Wait for threadFinishedEVent abandoned");
	}
	if (waitResult == WAIT_FAILED) {
		DebugPrintf(L"Wait for threadFinishedEVent Error: %d", GetLastError());
	}

	// Close all the handles
	int closeResult;

	closeResult = CloseHandle(threadFinishedEvent);
	if (closeResult == 0) {
		DebugPrintf(L"Close threadFinsishedEvent Error: %d", GetLastError());
	}

	closeResult = CloseHandle(frameReceivedEvent);
	if (closeResult == 0) {
		DebugPrintf(L"Close frameReceivedEvent Error: %d", GetLastError());
	}
	closeResult = CloseHandle(threadHandle);
	if (closeResult == 0) {
		DebugPrintf(L"Close threadHandle Error: %d", GetLastError());
	}

	TouCAN_deinit();
	Sleep(50);
	Toucan_winusb_deinit(&deviceData);

	return TWOCAN_RESULT_SUCCESS;
}

DllExport int ReadAdapter(byte* frame) {
	DebugPrintf(L"TouCAN ReadAdapter\n");
	// Save the pointer to the Can Frame buffer
	canFramePtr = frame;

	// Indicate thread is in running state
	isRunning = TRUE;

	// Start the read thread
	threadHandle = CreateThread(NULL, 0, ReadThread, NULL, 0, &threadId);

	if (threadHandle != NULL) {
		return TWOCAN_RESULT_SUCCESS;
	}
	// Fatal Error
	isRunning = FALSE;
	DebugPrintf(L"Read thread failed: %d (%d)\n", threadId, GetLastError());
	return SET_ERROR(TWOCAN_RESULT_FATAL, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_CREATE_THREAD_HANDLE);
}

DllExport int WriteAdapter(const unsigned int id, const int dataLength, byte* data) {
	//DebugPrintf(L"TouCAN WriteAdapter\n");

	BOOL status;

	status = TouCAN_write(id, dataLength, data);
	if (status == TRUE) {
		return TWOCAN_RESULT_SUCCESS;
	}
	else {
		DebugPrintf(L"Transmit frame failed: %d\n", status);
		return SET_ERROR(TWOCAN_RESULT_ERROR, TWOCAN_SOURCE_DRIVER, TWOCAN_ERROR_TRANSMIT_FAILURE);
	}
}

//
// Read thread, reads CAN Frames from Rusoku Toucan device, if a valid frame is received,
// parse the frame into the correct format and notify the caller
// Upon exit, returns TWOCAN_RESULT_SUCCESS as Thread Exit Code
//

DWORD WINAPI ReadThread(LPVOID lParam){

	DebugPrintf(L"TouCAN ReadThread\n");

	UINT8	index = 0;
	UINT8	FrameCounter = 0;

	struct _CAN_MSG
	{
		UINT32  flags;
		UINT32  id;
		UINT8	sizeData;
		UINT8   data[64];
		UINT32	timestamp;
	}msg;

	DWORD   mutexResult;

	while (isRunning) {		
		if (TouCAN_read(RxFrame, sizeof(RxFrame), &RxFrameTransferd) == TRUE) {
			//DebugPrintf(L"Received total bytes: %d\n", RxFrameTransferd);

			// Check FrameCounter number from total USB length
			switch (RxFrameTransferd)
			{
			case	18:
				FrameCounter = 1;
				break;
			case	36:
				FrameCounter = 2;
				break;
			case	54:
				FrameCounter = 3;
				break;
			default:
				FrameCounter = 0;
				break;
			}
			//DebugPrintf(L"Received frames: %d\n", FrameCounter);

			if (FrameCounter == 0)
				continue;

			index = 0;
			for (UINT8 x = 0; x < FrameCounter; x++)
			{
				msg.flags = (((UINT32)RxFrame[index++]) & 0x000000ff);

				msg.id = (((UINT32)RxFrame[index++] << 24) & 0x1f000000);
				msg.id |= (((UINT32)RxFrame[index++] << 16) & 0x00ff0000);
				msg.id |= (((UINT32)RxFrame[index++] << 8) & 0x0000ff00);
				msg.id |= (((UINT32)RxFrame[index++]) & 0x000000ff);

				msg.sizeData = RxFrame[index++];
				memcpy_s((UINT8*)msg.data, 8, &RxFrame[index], 8);
				index += 8;

				msg.timestamp = (((UINT32)RxFrame[index++] << 24) & 0xff000000);
				msg.timestamp |= (((UINT32)RxFrame[index++] << 16) & 0x00ff0000);
				msg.timestamp |= (((UINT32)RxFrame[index++] << 8) & 0x0000ff00);
				msg.timestamp |= (((UINT32)RxFrame[index++]) & 0x000000ff);

				// We are interested in CAN Extended frames only
				if (msg.flags != CANAL_IDFLAG_EXTENDED) {
					continue;
				}
			
				// Make sure we can get a lock on the buffer
				mutexResult = WaitForSingleObject(frameReceivedMutex, 200);

				if (mutexResult == WAIT_OBJECT_0) {

					// Convert id (long) to TwoCan header format (byte array)
					canFramePtr[3] = (msg.id >> 24) & 0xFF;
					canFramePtr[2] = (msg.id >> 16) & 0xFF;
					canFramePtr[1] = (msg.id >> 8) & 0xFF;
					canFramePtr[0] =  msg.id & 0xFF;

					// Copy the CAN data
					memcpy(&canFramePtr[4], msg.data, msg.sizeData);

					// Release the lock
					ReleaseMutex(frameReceivedMutex);

					// Notify the caller
					if (!SetEvent(frameReceivedEvent)) {						
						// Non fatal error
						DebugPrintf(L"Set Event Error: %d\n", GetLastError());
					}
				}
				else {
					// Non fatal error
					DebugPrintf(L"Adapter Mutex: %d -->%d\n", mutexResult, GetLastError());
				}

			}
		}
	}

	SetEvent(threadFinishedEvent);
	DebugPrintf(L"TouCAN ReadThread exit\n");
	ExitThread(TWOCAN_RESULT_SUCCESS);
}

