/**
 * \cond INCLUDE_PRIVATE
 * \file
 *
 * \section LICENSE
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Inertial Labs, Inc(TM)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 * This file supplies the cross-platform services when on a windows machine.
 */



#include <stdlib.h>
#include <fcntl.h>
#include <windows.h>
#include <errno.h>
#include <sysinfoapi.h>
#include <string.h>
#include "InertialLabs_services.h"
#include "IL_errorCodes.h"

#ifdef defined(__linux__) || defined(__QNXNTO__)

#include <unistd.h>
#include <termios.h>
#endif


#define MAX_KEY_LENGTH			255
#define MAX_PORT_NAME_LENGTH	30

 /* Private type declarations. */

typedef struct {
	IL_THREAD_START_ROUTINE	startRoutine;
	void*					routineData;
} IlThreadStartData;

/* Private function declarations. */

DWORD _stdcall inertial_thread_startRoutine(LPVOID lpThreadParameter);

IL_ERROR_CODE inertial_convertNativeToIlErrorCode(int nativeErrorCode);

bool inertial_isOsWinXp();

IL_ERROR_CODE inertial_getComPortRegistryKey(
	char const* portName,
	const wchar_t* controlSetName,
	PHKEY key);

/* Private variables. */
double _pcFreq = 0.0;
__int64 _counterStart = -1;

bool  inertial_isOsWinXp()
{
	DWORD dwVersion = 0;
	DWORD dwMajorVersion = 0;
	DWORD dwMinorVersion = 0;

	//dwVersion = GetVersion();
	dwVersion = 1.1;
	dwMajorVersion = (DWORD)(LOBYTE(LOWORD(dwVersion)));
	dwMinorVersion = (DWORD)(HIBYTE(LOWORD(dwVersion)));

	return dwMajorVersion == 5 && dwMinorVersion >= 1;
}

IL_ERROR_CODE inertial_thread_startNew(IL_HANDLE* newThreadHandle, IL_THREAD_START_ROUTINE startRoutine, void* routineData)
{
	IlThreadStartData* data = (IlThreadStartData*)malloc(sizeof(IlThreadStartData));

	data->startRoutine = startRoutine;
	data->routineData = routineData;

	*newThreadHandle = CreateThread(NULL, 0, inertial_thread_startRoutine, data, 0, NULL);

	if (*newThreadHandle == NULL)
		return  inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_open(
	IL_HANDLE* newComPortHandle,
	char const* portName,
	unsigned int baudrate)
{
	DCB config;
	char* preName = "\\\\.\\";
	char* fullName;
	COMMTIMEOUTS comTimeOut;

	fullName = (char*)malloc(strlen(preName) + strlen(portName) + 1);
	//strcpy_s(fullName, strlen(preName), preName);
	strcpy(fullName, preName);
	//strcat_s(fullName, strlen(portName), portName);
	strcpy(fullName, portName);
	*newComPortHandle = CreateFileA(fullName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	free(fullName);

	if (*newComPortHandle == INVALID_HANDLE_VALUE)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	/* Set the state of the COM port. */
	if (!GetCommState(*newComPortHandle, &config))
		return inertial_convertNativeToIlErrorCode(GetLastError());

	config.BaudRate = baudrate;
	config.StopBits = ONESTOPBIT;
	config.Parity = NOPARITY;
	config.ByteSize = 8;
	if (!SetCommState(*newComPortHandle, &config))
		return inertial_convertNativeToIlErrorCode(GetLastError());

	comTimeOut.ReadIntervalTimeout = 0;
	comTimeOut.ReadTotalTimeoutMultiplier = 0;
	comTimeOut.ReadTotalTimeoutConstant = 1;
	comTimeOut.WriteTotalTimeoutMultiplier = 3;
	comTimeOut.WriteTotalTimeoutConstant = 2;
	if (!SetCommTimeouts(*newComPortHandle, &comTimeOut))
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_writeData(
	IL_HANDLE comPortHandle,
	char const* dataToWrite,
	unsigned int numOfBytesToWrite)
{
	DWORD numOfBytesWritten;
	BOOL result;

	result = WriteFile(comPortHandle, dataToWrite, numOfBytesToWrite, &numOfBytesWritten, NULL);

	if (!result)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	result = FlushFileBuffers(comPortHandle);

	if (!result)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_readData(
	IL_HANDLE comPortHandle,
	char* readBuffer,
	unsigned int numOfBytesToRead,
	unsigned int* numOfBytesActuallyRead)
{
	BOOL result;

	result = ReadFile(comPortHandle, readBuffer, numOfBytesToRead, (LPDWORD)numOfBytesActuallyRead, NULL);

	if (!result)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_close(
	IL_HANDLE comPortHandle)
{
	BOOL result;

	result = CloseHandle(comPortHandle);

	if (!result)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_getComPortRegistryKey(
	char const* portName,
	const wchar_t* controlSetName,
	PHKEY key)
{
	HKEY ftdiBusKey;
	long error;
	TCHAR systemClassName[MAX_PATH] = TEXT("");
	DWORD systemClassNameSize = MAX_PATH;
	TCHAR className[MAX_PATH] = TEXT("");
	DWORD classNameSize = MAX_PATH;
	DWORD numOfSubKeys = 0;
	DWORD maxSubKeyLength;
	DWORD maxClassNameSize;
	DWORD numOfKeyValues;
	DWORD maxValueNameSize;
	DWORD maxValueDataSize;
	DWORD securityDescriptor;
	FILETIME lastWriteTime;
	TCHAR ftdiBusKeyPath[MAX_PATH] = TEXT("SYSTEM\\");

	wcscat(ftdiBusKeyPath, controlSetName);
	wcscat(ftdiBusKeyPath, L"\\Enum\\FTDIBUS");

	/* Open the FTDIBUS on CurrentControlSet. */
	error = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		ftdiBusKeyPath,
		0,
		KEY_READ,
		&ftdiBusKey);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	/* Get all of the keys of the FTDIBUS key. */
	error = RegQueryInfoKey(
		ftdiBusKey,
		className,
		&classNameSize,
		NULL,
		&numOfSubKeys,
		&maxSubKeyLength,
		&maxClassNameSize,
		&numOfKeyValues,
		&maxValueNameSize,
		&maxValueDataSize,
		&securityDescriptor,
		&lastWriteTime);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	if (numOfSubKeys > 0)
	{
		int i;

		for (i = 0; i < numOfSubKeys; i++)
		{
			TCHAR subKeyNameToCheck[MAX_PATH] = TEXT("");
			TCHAR keyPortName[MAX_PATH] = TEXT("");
			TCHAR subKeyName[MAX_KEY_LENGTH] = TEXT("");
			DWORD subKeyNameSize = MAX_KEY_LENGTH;
			HKEY subKey;
			DWORD keyPortNameSize = MAX_PATH;
			char retrievedPortName[MAX_PORT_NAME_LENGTH];
			DWORD latencyTimerValue;
			DWORD latencyTimerValueSize = sizeof(DWORD);

			error = RegEnumKeyEx(
				ftdiBusKey,
				i,
				subKeyName,
				&subKeyNameSize,
				NULL,
				NULL,
				NULL,
				&lastWriteTime);

			if (error != ERROR_SUCCESS)
				return inertial_convertNativeToIlErrorCode(error);

			wcscpy(subKeyNameToCheck, ftdiBusKeyPath);
			wcscat(subKeyNameToCheck, L"\\");
			wcscat(subKeyNameToCheck, subKeyName);
			wcscat(subKeyNameToCheck, L"\\");
			wcscat(subKeyNameToCheck, L"\\0000\\Device Parameters");

			error = RegOpenKeyEx(
				HKEY_LOCAL_MACHINE,
				subKeyNameToCheck,
				0,
				KEY_QUERY_VALUE,
				&subKey);

			if (error != ERROR_SUCCESS)
				return inertial_convertNativeToIlErrorCode(error);

			error = RegQueryValueEx(
				subKey,
				L"PortName",
				NULL,
				NULL,
				(LPBYTE)keyPortName,
				&keyPortNameSize);

			if (error != ERROR_SUCCESS)
				return inertial_convertNativeToIlErrorCode(error);

			/* Let's see if this is the port we are looking for. */
			wcstombs(retrievedPortName, keyPortName, MAX_PORT_NAME_LENGTH);
			if (strcmp(retrievedPortName, portName) != 0)
				/* Not the port we are looking for. */
				continue;

			/* We found the port we are looking for! */
			error = RegOpenKeyEx(
				HKEY_LOCAL_MACHINE,
				subKeyNameToCheck,
				0,
				KEY_READ,
				key);

			if (error != ERROR_SUCCESS)
				return inertial_convertNativeToIlErrorCode(error);

			return ILERR_NO_ERROR;
		}
	}

	/* We must not have been able to find the COM port settings. */
	return ILERR_FILE_NOT_FOUND;
}

IL_ERROR_CODE inertial_comPort_isOptimized(
	char const* portName,
	bool* isOptimized)
{
	HKEY systemKey;
	long error;
	DWORD systemNumOfSubKeys = 0;
	int i = 0;

	*isOptimized = false;

	/* Get the list of ControlSets. */
	error = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		L"SYSTEM",
		0,
		KEY_READ,
		&systemKey);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	error = RegQueryInfoKey(
		systemKey,
		NULL,
		NULL,
		NULL,
		&systemNumOfSubKeys,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	/* Go through each ControlSet00X. */
	for (i = 0; i < systemNumOfSubKeys; i++)
	{
		TCHAR controlSetName[MAX_KEY_LENGTH] = TEXT("");
		DWORD controlSetNameSize = MAX_KEY_LENGTH;
		HKEY comPortKey;
		DWORD latencyTimerValue;
		DWORD latencyTimerValueSize = sizeof(DWORD);

		error = RegEnumKeyEx(
			systemKey,
			i,
			controlSetName,
			&controlSetNameSize,
			NULL,
			NULL,
			NULL,
			NULL);

		if (error != ERROR_SUCCESS)
			return inertial_convertNativeToIlErrorCode(error);

		/* See if this matches our ControlSet00X pattern. */
		if (wcsncmp(L"ControlSet", controlSetName, 10) != 0)
			/* Not what we are looking for. */
			continue;

		error = inertial_getComPortRegistryKey(
			portName,
			controlSetName,
			&comPortKey);

		if (error == ILERR_FILE_NOT_FOUND)
			/* No registry entry. */
			continue;

		if (error != ILERR_NO_ERROR && error != ILERR_FILE_NOT_FOUND)
			return error;

		/* Check the value of the LatencyTimer field. */
		error = RegQueryValueEx(
			comPortKey,
			L"LatencyTimer",
			NULL,
			NULL,
			(LPBYTE)&latencyTimerValue,
			&latencyTimerValueSize);

		if (error != ERROR_SUCCESS)
			return inertial_convertNativeToIlErrorCode(error);

		if (latencyTimerValue != 1)
			/* We have already inialized isOptimized to false. */
			return ILERR_NO_ERROR;
	}

	/* If we got here, either we did not find any registry entries for the COM
	   port (possibly indicating this is not an FTDI USB Virtual COM port) or
	   the registry entries we found were already optimized. */

	*isOptimized = true;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_optimize(
	char const* portName)
{
	HKEY systemKey;
	long error;
	DWORD systemNumOfSubKeys = 0;
	int i = 0;
	DWORD optimizedLatencyTimerValue = 1;
	bool haveFoundAtLeastOneRegistryEntryForComPort = false;

	/* Get the list of ControlSets. */
	error = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		L"SYSTEM",
		0,
		KEY_READ,
		&systemKey);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	error = RegQueryInfoKey(
		systemKey,
		NULL,
		NULL,
		NULL,
		&systemNumOfSubKeys,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL);

	if (error != ERROR_SUCCESS)
		return inertial_convertNativeToIlErrorCode(error);

	/* Go through each ControlSet00X. */
	for (i = 0; i < systemNumOfSubKeys; i++)
	{
		TCHAR controlSetName[MAX_KEY_LENGTH] = TEXT("");
		DWORD controlSetNameSize = MAX_KEY_LENGTH;
		HKEY comPortKey, comPortKeyWriteAccess;
		DWORD latencyTimerValue;
		DWORD latencyTimerValueSize = sizeof(DWORD);

		error = RegEnumKeyEx(
			systemKey,
			i,
			controlSetName,
			&controlSetNameSize,
			NULL,
			NULL,
			NULL,
			NULL);

		if (error != ERROR_SUCCESS)
			return inertial_convertNativeToIlErrorCode(error);

		/* See if this matches our ControlSet00X pattern. */
		if (wcsncmp(L"ControlSet", controlSetName, 10) != 0)
			/* Not what we are looking for. */
			continue;

		error = inertial_getComPortRegistryKey(
			portName,
			controlSetName,
			&comPortKey);

		if (error != ILERR_NO_ERROR && error != ILERR_FILE_NOT_FOUND)
			if (error != ILERR_NO_ERROR && error != ILERR_FILE_NOT_FOUND)
				return error;

		haveFoundAtLeastOneRegistryEntryForComPort = true;

		error = RegOpenKeyEx(
			comPortKey,
			NULL,
			0,
			KEY_SET_VALUE,
			&comPortKeyWriteAccess);

		if (error != ERROR_SUCCESS)
			return inertial_convertNativeToIlErrorCode(error);

		error = RegSetValueEx(
			comPortKeyWriteAccess,
			L"LatencyTimer",
			0,
			REG_DWORD,
			(uint8_t*)&optimizedLatencyTimerValue,
			sizeof(DWORD));

		if (error != ERROR_SUCCESS)
			return inertial_convertNativeToIlErrorCode(error);
	}

	if (haveFoundAtLeastOneRegistryEntryForComPort)
		return ILERR_NO_ERROR;
	else
		/* Did not find any registry entries for the COM port. */
		return ILERR_FILE_NOT_FOUND;
}

IL_ERROR_CODE inertial_event_create(
	IL_HANDLE* newEventHandle)
{
	*newEventHandle = CreateEvent(NULL, FALSE, FALSE, NULL);

	if (*newEventHandle == NULL)
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_event_waitFor(
	IL_HANDLE eventHandle, int timeout)
{
	DWORD result;

	result = WaitForSingleObject(eventHandle, timeout);

	if (result == WAIT_OBJECT_0)
		return ILERR_NO_ERROR;
	if (result == WAIT_TIMEOUT)
		return ILERR_TIMEOUT;
	if (result == WAIT_FAILED)
		return inertial_convertNativeToIlErrorCode(result);

	return ILERR_UNKNOWN_ERROR;
}

IL_ERROR_CODE inertial_event_signal(
	IL_HANDLE eventHandle)
{
	if (!SetEvent(eventHandle))
		return inertial_convertNativeToIlErrorCode(GetLastError());

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_initialize(
	IL_CRITICAL_SECTION* criticalSection)
{
	InitializeCriticalSection(criticalSection);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_enter(
	IL_CRITICAL_SECTION* criticalSection)
{
	EnterCriticalSection(criticalSection);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_leave(
	IL_CRITICAL_SECTION* criticalSection)
{
	LeaveCriticalSection(criticalSection);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_dispose(
	IL_CRITICAL_SECTION* criticalSection)
{
	DeleteCriticalSection(criticalSection);

	return ILERR_NO_ERROR;
}

DWORD _stdcall inertial_thread_startRoutine(
	LPVOID lpThreadParameter)
{
	IlThreadStartData data;

	data = *((IlThreadStartData*)lpThreadParameter);

	free(lpThreadParameter);

	/* Call the user's thread routine. */
	data.startRoutine(data.routineData);

	return 0;
}

IL_ERROR_CODE inertial_convertNativeToIlErrorCode(
	int nativeErrorCode)
{
	switch (nativeErrorCode)
	{
	case WAIT_TIMEOUT:
		return ILERR_TIMEOUT;
	case ERROR_FILE_NOT_FOUND:
		return ILERR_FILE_NOT_FOUND;
	case ERROR_ACCESS_DENIED:
		return ILERR_PERMISSION_DENIED;
	case ERROR_INVALID_PARAMETER:
		return ILERR_INVALID_VALUE;
	default:
		return ILERR_UNKNOWN_ERROR;
	}
}

IL_ERROR_CODE inertial_sleepInMs(
	unsigned int numOfMillisecondsToSleep)
{
	Sleep(numOfMillisecondsToSleep);

	return ILERR_NO_ERROR;
}

void inertial_startMsTimer()
{
	LARGE_INTEGER li;

	if (!QueryPerformanceFrequency(&li))
		/* The hardware must not support a high-resolution performance counter. */
		return;

	_pcFreq = ((double)li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);

	_counterStart = li.QuadPart;
}

double inertial_stopMsTimer()
{
	LARGE_INTEGER li;
	double result;

	if (_counterStart == -1)
		return -1.0;

	QueryPerformanceCounter(&li);

	result = (((double)li.QuadPart) - _counterStart) / _pcFreq;

	_counterStart = -1;

	return result;
}
/** \endcond */
