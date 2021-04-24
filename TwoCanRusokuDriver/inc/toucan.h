//
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
//


#ifndef _TWOCAN_TOUCAN
#define _TWOCAN_TOUCAN

#include "..\common\inc\twocandriver.h"
#include "..\inc\toucan_hardware.h"

// Win32 functions
#define WINDOWS_LEAN_AND_MEAN
#include <windows.h>

// 'C' runtime functions
#include <stdio.h>

#define DllExport __declspec( dllexport )

#ifdef __cplusplus
extern "C" {
#endif

	DllExport char* DriverName(void);
	DllExport char* DriverVersion(void);
	DllExport char* ManufacturerName(void);
	DllExport int OpenAdapter(void);
	DllExport int CloseAdapter(void);
	DllExport int ReadAdapter(byte* frame);
	DllExport int WriteAdapter(const unsigned int id, const int dataLength, byte* data);

#ifdef __cplusplus
}
#endif

DWORD WINAPI ReadThread(LPVOID lParam);

#endif