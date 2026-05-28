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
 * This file supplies the cross-platform services when on a Linux machine.
 */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include "InertialLabs_services.h"
#include "IL_errorCodes.h"

/* Private type declarations. ************************************************/
typedef struct {
	IL_THREAD_START_ROUTINE	startRoutine;
	void*					routineData;
} IlThreadStartData;

typedef struct {
	pthread_mutex_t		mutex;
	pthread_cond_t		condition;
	IL_BOOL				isTriggered;
} IlConditionAndMutex;

/* Private function declarations. ********************************************/
void* inertial_thread_startRoutine(void* threadStartData);
IL_ERROR_CODE inertial_convertNativeToIlErrorCode(int nativeErrorCode);

/**
 * \brief Determines what the baudrate flag should be for the provide baudrate.
 *
 * \param[in]	baudrate	Desired baudrate.
 * \return The appropriate baudrate flag to set in the termios.c_cflag field to
 * get the desired baudrate. If the provided baudrate is invalid, the value B0
 * will be returned.
 */
tcflag_t inertial_determineBaudrateFlag(unsigned int baudrate);

IL_ERROR_CODE inertial_thread_startNew(IL_HANDLE* newThreadHandle, IL_THREAD_START_ROUTINE startRoutine, void* routineData)
{
	int errorCode;

	IlThreadStartData* data = (IlThreadStartData*) malloc(sizeof(IlThreadStartData));
	
	data->startRoutine = startRoutine;
	data->routineData = routineData;

	errorCode = pthread_create(&newThreadHandle->pThreadHandle, NULL, inertial_thread_startRoutine, data);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_open(IL_HANDLE* newComPortHandle, char const* portName, unsigned int baudrate)
{
	struct termios portSettings;
	int portFd = -1;
	tcflag_t baudrateFlag;

	portFd = open(portName, O_RDWR | O_NOCTTY);
	if (portFd == -1)
		return inertial_convertNativeToIlErrorCode(errno);

	/* clear struct for new port settings */
	memset(&portSettings, 0, sizeof(portSettings));

	baudrateFlag = inertial_determineBaudrateFlag(baudrate);
	if (baudrateFlag == B0)
		return ILERR_UNKNOWN_ERROR;

	/* Set baudrate, 8n1, no modem control, enable receiving characters. */
	portSettings.c_cflag = baudrateFlag | CS8 | CLOCAL | CREAD;

	portSettings.c_iflag = IGNPAR;		/* Ignore bytes with parity errors. */
	portSettings.c_oflag = 0;			/* Enable raw data output. */

	portSettings.c_cc[VTIME]    = 0;	/* Do not use inter-character timer. */
	portSettings.c_cc[VMIN]     = 0;	/* Block on reads until 0 character is received. */

    /* Clear the COM port buffers. */
	if (tcflush(portFd, TCIFLUSH) != 0)
		return inertial_convertNativeToIlErrorCode(errno);

	if (tcsetattr(portFd, TCSANOW, &portSettings) != 0)
		return inertial_convertNativeToIlErrorCode(errno);

	newComPortHandle->comPortHandle = portFd;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_writeData(IL_HANDLE comPortHandle, unsigned char  dataToWrite[], unsigned int numOfBytesToWrite)
{
	int errorCode;

	errorCode = write(comPortHandle.comPortHandle, dataToWrite, numOfBytesToWrite);

	if (errorCode == -1)
		return inertial_convertNativeToIlErrorCode(errno);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_readData(IL_HANDLE comPortHandle, unsigned char readBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	int numOfBytesRead;

	*numOfBytesActuallyRead = 0;

	numOfBytesRead = read(comPortHandle.comPortHandle, readBuffer, numOfBytesToRead);
	
	if (numOfBytesRead == -1)
		return inertial_convertNativeToIlErrorCode(errno);

	*numOfBytesActuallyRead = (unsigned int) numOfBytesRead;

	
	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_comPort_close(IL_HANDLE comPortHandle)
{
	if (close(comPortHandle.comPortHandle) == -1)
		return inertial_convertNativeToIlErrorCode(errno);

	return ILERR_NO_ERROR;
}

tcflag_t inertial_determineBaudrateFlag(unsigned int baudrate)
{
	switch (baudrate) {
		case 9600:		return B9600;
		case 19200:		return B19200;
		case 38400:		return B38400;
		case 57600:		return B57600;
		case 115200:	return B115200;
		case 230400:	return B230400;
		case 460800:	return B460800;
		case 921600:	return B921600;
		default:		return B0;
	}
}

IL_ERROR_CODE inertial_event_create(IL_HANDLE* newEventHandle)
{
	int errorCode;
	IlConditionAndMutex* cm;

	cm = (IlConditionAndMutex*) malloc(sizeof(IlConditionAndMutex));
	
	errorCode = pthread_mutex_init(&cm->mutex, NULL);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);
		
	errorCode = pthread_cond_init(&cm->condition, NULL);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	cm->isTriggered = IL_FALSE;

	newEventHandle->conditionAndMutexStruct = cm;
		
	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_event_waitFor(IL_HANDLE eventHandle, int timeout)
{
	int errorCode, loopErrorCode;
	IlConditionAndMutex* cm;
	struct timespec delta;
	struct timespec abstime;
	
	// Compute our timeout.
	if (timeout != -1) {

		//TODO: implement CLOCK_REALTIME linux macro here
		clock_gettime(CLOCK_REALTIME, &abstime);

		int nano = abstime.tv_nsec + (timeout % 1000) * 1000000;

		delta.tv_nsec = nano % 1000000000;
		delta.tv_sec = abstime.tv_sec + timeout / 1000 + nano / 1000000000;
	}

	cm = (IlConditionAndMutex*) eventHandle.conditionAndMutexStruct;

	errorCode = pthread_mutex_lock(&cm->mutex);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	while (!cm->isTriggered) {
		
		if (timeout == -1) {
			loopErrorCode = pthread_cond_wait(&cm->condition, &cm->mutex);

			if (loopErrorCode != 0)
				return inertial_convertNativeToIlErrorCode(loopErrorCode);
		}
		else {
			loopErrorCode = pthread_cond_timedwait(&cm->condition, &cm->mutex, &delta);

			if (loopErrorCode == ETIMEDOUT) {
				cm->isTriggered = IL_FALSE;
				errorCode = pthread_mutex_unlock(&cm->mutex);
	
				return ILERR_TIMEOUT;
			}
		}
	}

	cm->isTriggered = IL_FALSE;

	errorCode = pthread_mutex_unlock(&cm->mutex);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_event_signal(IL_HANDLE eventHandle)
{
	int errorCode;
	IlConditionAndMutex* cm;

	cm = (IlConditionAndMutex*) eventHandle.conditionAndMutexStruct;

	errorCode = pthread_mutex_lock(&cm->mutex);

	cm->isTriggered = IL_TRUE;

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	errorCode = pthread_cond_signal(&cm->condition);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	errorCode = pthread_mutex_unlock(&cm->mutex);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

void* inertial_thread_startRoutine(void* threadStartData)
{
	IlThreadStartData* data;

	data = (IlThreadStartData*) threadStartData;

	/* Call the user's thread routine. */
	data->startRoutine(data->routineData);

	return 0;
}

IL_ERROR_CODE inertial_convertNativeToIlErrorCode(int nativeErrorCode)
{
	switch (nativeErrorCode) {

		case ENOENT:
			return ILERR_FILE_NOT_FOUND;

		default:
			return ILERR_UNKNOWN_ERROR;
	}
}

IL_ERROR_CODE inertial_sleepInMs(unsigned int numOfMillisecondsToSleep)
{
	usleep(numOfMillisecondsToSleep * 1000);

	return ILERR_NO_ERROR;
}



IL_ERROR_CODE inertial_criticalSection_initialize(IL_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_init(criticalSection, NULL);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);
		
	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_enter(IL_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_lock(criticalSection);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_leave(IL_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_unlock(criticalSection);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_criticalSection_dispose(IL_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_destroy(criticalSection);

	if (errorCode != 0)
		return inertial_convertNativeToIlErrorCode(errorCode);

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE inertial_clean_mallocs()
{
	//int errorCode;

	return ILERR_NO_MEMORY_ERROR;
}

/** \endcond */
