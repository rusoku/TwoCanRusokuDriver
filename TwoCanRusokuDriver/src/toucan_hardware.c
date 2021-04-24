// 
// Parts of source code that communicates with a USB device using WinUSB are used from Microsoft Corporation examples
//
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

#include "..\inc\toucan_hardware.h"
#include "..\common\inc\twocanerror.h"

DEVICE_DATA           deviceData;
HRESULT               hr;
USB_DEVICE_DESCRIPTOR deviceDesc;
BOOL                  noDevice;
ULONG                 lengthReceived;

HRESULT Toucan_winusb_init( DEVICE_DATA *DeviceData, BOOL *FailureDeviceNotFound )
{
    HRESULT hr = S_OK;
    BOOL    bResult;

    DeviceData->HandlesOpen = FALSE;

    hr = RetrieveDevicePath(DeviceData->DevicePath, sizeof(DeviceData->DevicePath), FailureDeviceNotFound);

    if (FAILED(hr)) {
        DebugPrintf(L"RetrieveDevicePath failed (%ld)\n", hr);
        return hr;
    }

    DebugPrintf(L"TouCAN RetrieveDevicePath: %s)\n", DeviceData->DevicePath);

    DeviceData->DeviceHandle = CreateFile(DeviceData->DevicePath,
        GENERIC_WRITE | GENERIC_READ,
        FILE_SHARE_WRITE | FILE_SHARE_READ,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (INVALID_HANDLE_VALUE == DeviceData->DeviceHandle) {

        hr = HRESULT_FROM_WIN32(GetLastError());
        return hr;
    }

    bResult = WinUsb_Initialize(DeviceData->DeviceHandle,
        &DeviceData->WinusbHandle);

    if (FALSE == bResult) {

        hr = HRESULT_FROM_WIN32(GetLastError());
        CloseHandle(DeviceData->DeviceHandle);
        return hr;
    }

    DeviceData->HandlesOpen = TRUE;
    return hr;
}

VOID Toucan_winusb_deinit(DEVICE_DATA *DeviceData)
{
    if (FALSE == DeviceData->HandlesOpen) {
        return;
    }

    WinUsb_Free(DeviceData->WinusbHandle);
    CloseHandle(DeviceData->DeviceHandle);
    DeviceData->HandlesOpen = FALSE;

    return;
}

HRESULT RetrieveDevicePath(LPTSTR DevicePath, ULONG  BufLen, BOOL *FailureDeviceNotFound)
{
    CONFIGRET cr = CR_SUCCESS;
    HRESULT   hr = S_OK;
    PTSTR     DeviceInterfaceList = NULL;
    ULONG     DeviceInterfaceListLength = 0;

    if (NULL != FailureDeviceNotFound) {

        *FailureDeviceNotFound = FALSE;
    }

    //
    // Enumerate all devices exposing the interface. Do this in a loop
    // in case a new interface is discovered while this code is executing,
    // causing CM_Get_Device_Interface_List to return CR_BUFFER_SMALL.
    //
    do {
        cr = CM_Get_Device_Interface_List_Size(&DeviceInterfaceListLength,
            (LPGUID)&GUID_DEVINTERFACE_WinUsbF4FS1,
            NULL,
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

        if (cr != CR_SUCCESS) {
            hr = HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_INVALID_DATA));
            break;
        }

        DeviceInterfaceList = (PTSTR)HeapAlloc(GetProcessHeap(),
            HEAP_ZERO_MEMORY,
            DeviceInterfaceListLength * sizeof(TCHAR));

        if (DeviceInterfaceList == NULL) {
            hr = E_OUTOFMEMORY;
            break;
        }

        cr = CM_Get_Device_Interface_List((LPGUID)&GUID_DEVINTERFACE_WinUsbF4FS1,
            NULL,
            DeviceInterfaceList,
            DeviceInterfaceListLength,
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

        if (cr != CR_SUCCESS) {
            HeapFree(GetProcessHeap(), 0, DeviceInterfaceList);

            if (cr != CR_BUFFER_SMALL) {
                hr = HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_INVALID_DATA));
            }
        }
    } while (cr == CR_BUFFER_SMALL);

    if (FAILED(hr)) {
        return hr;
    }

    //
    // If the interface list is empty, no devices were found.
    //
    if (*DeviceInterfaceList == TEXT('\0')) {
        if (NULL != FailureDeviceNotFound) {
            *FailureDeviceNotFound = TRUE;
        }

        hr = HRESULT_FROM_WIN32(ERROR_NOT_FOUND);
        HeapFree(GetProcessHeap(), 0, DeviceInterfaceList);

        return hr;
    }

    //
    // Give path of the first found device interface instance to the caller. CM_Get_Device_Interface_List ensured
    // the instance is NULL-terminated.
    //
    hr = StringCbCopy(DevicePath, BufLen, DeviceInterfaceList);
    HeapFree(GetProcessHeap(), 0, DeviceInterfaceList);

    return hr;
}

// NMEA2000 CAN bus speed: 250 kbit
//sampling point 75%
//m_Brp = 10;
//m_Tseg1 = 14;
//m_Tseg2 = 5;
//m_Sjw = 4;
BOOL TouCAN_init(void)
{
    UINT8		m_Tseg1 = 14;
    UINT8		m_Tseg2 = 5;
    UINT8		m_Sjw = 4;
    UINT16		m_Brp = 10;
    UINT32		m_OptionFlag = 0;

    UCHAR	data[64];
    ULONG	Transfered;
    UINT8	res;
    WINUSB_SETUP_PACKET SetupPacket;
    ULONG	timeout = 500;

    // Transmit frame timeout: 500mS
    WinUsb_SetPipePolicy(deviceData.WinusbHandle, 0x01, PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);
    WinUsb_SetPipePolicy(deviceData.WinusbHandle, 0x01, RAW_IO, 0, 0);

    // Receive frame timeout : 500mS
    WinUsb_SetPipePolicy(deviceData.WinusbHandle, 0x81, PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);
    WinUsb_SetPipePolicy(deviceData.WinusbHandle, 0x81, RAW_IO, 0, 0);

    SetupPacket.RequestType = USB_HOST_TO_DEVICE | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = TouCAN_CAN_INTERFACE_INIT;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 9;
    // tseg1
    data[0] = m_Tseg1;
    // tseg2
    data[1] = m_Tseg2;
    // sjw
    data[2] = m_Sjw;
    // Brp

    data[3] = (UINT8)((m_Brp >> 8) & 0xFF);
    data[4] = (UINT8)(m_Brp & 0xFF);

    // flags
    data[5] = (UINT8)((m_OptionFlag >> 24) & 0xFF);
    data[6] = (UINT8)((m_OptionFlag >> 16) & 0xFF);
    data[7] = (UINT8)((m_OptionFlag >> 8) & 0xFF);
    data[8] = (UINT8)(m_OptionFlag & 0xFF);

    // TouCAN_CAN_INTERFACE_INIT
    if (WinUsb_ControlTransfer(deviceData.WinusbHandle, SetupPacket, &data[0], 9, &Transfered, NULL) != TRUE)
        return	FALSE;

    if (TouCAN_get_last_error_code(&res) != TRUE)
        return FALSE;

    if (res != TouCAN_RETVAL_OK)
        return FALSE;

    return TRUE;
}

BOOL TouCAN_deinit(void)
{
    WINUSB_SETUP_PACKET SetupPacket;
    UINT8	res;

    SetupPacket.RequestType = USB_HOST_TO_DEVICE | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = TouCAN_CAN_INTERFACE_DEINIT;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 0;

    // TouCAN_CAN_INTERFACE_DEINIT
    if (WinUsb_ControlTransfer(deviceData.WinusbHandle, SetupPacket, NULL, 0, NULL, NULL) != TRUE)
        return	FALSE;

    if (TouCAN_get_last_error_code(&res) != TRUE)
        return FALSE;

    if (res != TouCAN_RETVAL_OK)
        return FALSE;

    return	TRUE;
}

BOOL TouCAN_start(void)
{
    WINUSB_SETUP_PACKET SetupPacket;
    UINT8	res;

    SetupPacket.RequestType = USB_HOST_TO_DEVICE | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = TouCAN_CAN_INTERFACE_START;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 0;

    // TouCAN_CAN_INTERFACE_START
    if (WinUsb_ControlTransfer(deviceData.WinusbHandle, SetupPacket, NULL, 0, NULL, NULL) != TRUE)
        return	FALSE;

    if (TouCAN_get_last_error_code(&res) != TRUE)
        return FALSE;

    if (res != TouCAN_RETVAL_OK)
        return FALSE;

    return TRUE;
}

BOOL TouCAN_stop(void)
{
    WINUSB_SETUP_PACKET SetupPacket;
    UINT8	res;

    SetupPacket.RequestType = USB_HOST_TO_DEVICE | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = TouCAN_CAN_INTERFACE_STOP;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 0;

    // TouCAN_CAN_INTERFACE_START
    if (WinUsb_ControlTransfer(deviceData.WinusbHandle, SetupPacket, NULL, 0, NULL, NULL) != TRUE)
        return	FALSE;

    if (TouCAN_get_last_error_code(&res) != TRUE)
        return FALSE;

    if (res != TouCAN_RETVAL_OK)
        return FALSE;

    return	TRUE;
}

BOOL TouCAN_get_last_error_code(UINT8* res)
{
    UINT8	LastErrorCode;
    ULONG	Transfered;
    WINUSB_SETUP_PACKET	SetupPacket;

    if (res == NULL)
        return FALSE;

    SetupPacket.RequestType = USB_DEVICE_TO_HOST | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = TouCAN_GET_LAST_ERROR_CODE;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 1;

    if (WinUsb_ControlTransfer(deviceData.WinusbHandle, SetupPacket, &LastErrorCode, 1, &Transfered, NULL) != TRUE)
        return	FALSE;

    if ((LastErrorCode != HAL_OK) & (Transfered != 1))
        return	FALSE;

    *res = LastErrorCode;

    return	TRUE;
}


BOOL TouCAN_write(const unsigned int id, const int dataLength, byte* data) {

    UINT8   TxDataBuf[64];
    ULONG	Transfered;
    UINT8	index = 0;

    // CAN frame flags
    TxDataBuf[index++] = (UINT8)CANAL_IDFLAG_EXTENDED; // Extended message id (29-bit)
    TxDataBuf[index++] = (UINT8)(id >> 24);
    TxDataBuf[index++] = (UINT8)(id >> 16);
    TxDataBuf[index++] = (UINT8)(id >> 8);
    TxDataBuf[index++] = (UINT8)id;

    // CAN msg data length
    TxDataBuf[index++] = dataLength;

    // CAN msg data
    for (UINT8 y = 0; y < 8; y++)    {
        TxDataBuf[index++] = data[y];
    }

    // CAN msg timestamp
    TxDataBuf[index++] = 0;
    TxDataBuf[index++] = 0;
    TxDataBuf[index++] = 0;
    TxDataBuf[index++] = 0;

    if (WinUsb_WritePipe(deviceData.WinusbHandle, 0x01, &TxDataBuf[0], index, &Transfered, NULL) == FALSE){
        DebugPrintf(L"TouCAN_write Error: %d", GetLastError());
        return FALSE;
    }
    return TRUE;
}


BOOL TouCAN_read( UINT8 *data, UINT16 dataLength, ULONG *Transfered ) {
    return WinUsb_ReadPipe(deviceData.WinusbHandle, 0x81, data, dataLength, Transfered, NULL);
}

