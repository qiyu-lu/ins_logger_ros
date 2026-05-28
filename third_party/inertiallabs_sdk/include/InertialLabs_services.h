/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to the cross-platform services for the
 * InerTialLabs C/C++ Library.
 */
#ifndef _INERTIALLABS_SERVICES_H_
#define _INERTIALLABS_SERVICES_H_

#if defined(_MSC_VER) && _MSC_VER <= 1500
 /* Visual Studio 2008 and earlier do not include the stdint.h header file. */
#include "IL_int.h"
#else
#include <stdint.h>
#endif

#if defined(WIN32) || defined(WIN64) 

 /* Disable some warnings for Visual Studio with -Wall. */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4668)
#pragma warning(disable:4820)
#pragma warning(disable:4255)
#endif

#include <windows.h>

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif

#if defined(__linux__) || defined(__QNXNTO__)
#include <pthread.h>
#endif

/* Determine the level of C support. */
#if __STDC__
#if defined (__STDC_VERSION__)
#if (__STDC_VERSION__ >= 1999901L)
#define C99
#endif
#endif
#endif

/* Define boolean type. */
#if defined (C99)
#include <stdbool.h>
#else
#if !defined (__cplusplus)
#if !defined (__GNUC__)
	/* _Bool builtin type is included in GCC. */
	/* ISO C Standard: 5.2.5 An object declared as type _Bool is large
	 * enough to store the values 0 and 1. */
	 //typedef int8_t _Bool;
#endif

/* ISO C Standard: 7.16 Boolean type */
#define bool _Bool
#define true 1
#define false 0
#define __bool_true_false_are_defined 1
#endif
#endif


#include "IL_errorCodes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IL_NULL ((void *) 0)
#define IL_TRUE ((IL_BOOL) 1)
#define IL_FALSE ((IL_BOOL) 0)

	typedef char IL_BOOL;

#if defined(WIN32) || defined(_x64) 
	typedef HANDLE				IL_HANDLE;
	typedef CRITICAL_SECTION	IL_CRITICAL_SECTION;
#elif defined(__linux__) || defined(__QNXNTO__)
	typedef	union {
		pthread_t			pThreadHandle;
		int					comPortHandle;
		pthread_mutex_t		mutexHandle;
		void*				conditionAndMutexStruct;
	} IL_HANDLE;
	typedef pthread_mutex_t	IL_CRITICAL_SECTION;
#endif


#define COMMAND_HEADER_SIZE				5
#define RESPONSE_BUILDER_BUFFER_SIZE	256
#define READ_BUFFER_SIZE				256
#define IL_MAX_COMMAND_SIZE				10
#define IL_MAX_RESPONSE_SIZE			256
#define IL_RESPONSE_MATCH_SIZE			10
#define NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_NOT_RECEIVING_ON_COM_PORT 	5
#define DEFAULT_TIMEOUT_IN_MS			5000



	typedef void *(*IL_THREAD_START_ROUTINE)(void*);

	/**
	 * \brief Creates a new thread.
	 *
	 * \param[out]	newThreadHandle		Handle of the newly created thread.
	 * \param[in]	startRoutine		Pointer to the routine the new thread should execute.
	 * \param[in]	routineData			Pointer to data that will be passed to the new thread's execution routine.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_thread_startNew(IL_HANDLE* newThreadHandle, IL_THREAD_START_ROUTINE startRoutine, void* routineData);

	/**
	 * \brief Opens a COM port.
	 *
	 * \param[out]	newComPortHandle	Handle to the newly opened COM port.
	 * \param[in]	portName			The name of the COM port to open.
	 * \param[in]	baudrate			The baudrate to communicate at.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_comPort_open(IL_HANDLE* newComPortHandle, char const* portName, unsigned int baudrate);

	/**
	 * \brief Write data out of a COM port.
	 *
	 * \param[in]	comPortHandle		Handle to an open COM port.
	 * \param[in]	dataToWrite			Pointer to array of bytes to write out the COM port.
	 * \param[in]	numOfBytesToWrite	The number of bytes to write from the dataToWrite pointer.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_comPort_writeData(IL_HANDLE comPortHandle, unsigned char dataToWrite[], unsigned int numOfBytesToWrite);

	/**
	 * \brief Reads data from a COM port. Will block temporarily for a short amount
	 * of time and then return if no data has been received.
	 *
	 * \param[in]	comPortHandle			Handle to an open COM port.
	 * \param[out]	readBuffer				Pointer to a buffer where data read from the COM port will be placed into.
	 * \param[in]	numOfBytesToRead		The number of bytes to attempt reading from the COM port.
	 * \param[out]	numOfBytesActuallyRead	The number of bytes actually read from the COM port during the read attempt.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_comPort_readData(IL_HANDLE comPortHandle, unsigned char readBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

	/**
	 * \brief Closes the COM port.
	 *
	 * \param[in]	comPortHandle	Handle to an open COM port.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_comPort_close(IL_HANDLE comPortHandle);

	/**
	 * \brief Creates a new event.
	 *
	 * \param[out]	newEventHandle	Returns the handle of the newly created event.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_event_create(IL_HANDLE* newEventHandle);

	/**
	 * \brief Causes the calling thread to wait on an event until the event is signalled.
	 *
	 * \param[in]	eventHandle		Handle to the event.
	 * \param[in]	timeout			The number of milliseconds to wait before the
	 * thread stops listening. -1 indicates that the wait time is inifinite. If a
	 * timeout does occur, the value ILERR_TIMEOUT will be retured.
	 *
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_event_waitFor(IL_HANDLE eventHandle, int timeout);

	/**
	 * \brief Puts the provided event into a signalled state.
	 *
	 * \param[in]	eventHandle		Handle to the event.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_event_signal(IL_HANDLE eventHandle);

	/**
	 * \brief Sleeps the current thread the specified number of milliseconds.
	 *
	 * \param[in]	numOfMillisecondsToSleep	The number of milliseconds to pause the current thread.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_sleepInMs(unsigned int numOfMillisecondsToSleep);

	/**
	 * \brief Initializes a new critical section object.
	 *
	 * \param[out]	criticalSection		Returns the newly initialized created critical control object.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_criticalSection_initialize(IL_CRITICAL_SECTION* criticalSection);

	/**
	 * \brief Attempt to enter a critical section.
	 *
	 * \param[in]	criticalSection		Pointer to the critical section control object.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_criticalSection_enter(IL_CRITICAL_SECTION* criticalSection);

	/**
	 * \brief Signals that the current executing thread is leaving the critical section.
	 *
	 * \param[in]	criticalSection		Pointer to the critical section control object.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_criticalSection_leave(IL_CRITICAL_SECTION* criticalSection);

	/**
	 * \brief Disposes and frees the resources associated with a critical section control object.
	 *
	 * \param[in]	criticalSection		Pointer to the critical section control object.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE inertial_criticalSection_dispose(IL_CRITICAL_SECTION* criticalSection);


#ifdef __cplusplus
}
#endif

#endif /* _INERTIALLABS_SERVICES_H_ */

/** \endcond */
