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
 * This file implements the functions for interfacing with a Inertial Labs device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include "InertialLabs_AHRS_10.h"
#include "IL_errorCodes.h"

#if defined(__linux__) || defined(__QNXNTO__)

#include <unistd.h>
#include <math.h>
#include <bits/stdio2.h>

#endif


typedef struct {
	/**
		 * Handle to the comPortServiceThread.
		 */
	IL_HANDLE				comPortServiceThreadHandle;
 
	/**
	 * Handle to the COM port.
	 */
	IL_HANDLE				comPortHandle;


	IL_HANDLE				waitForThreadToStopServicingComPortEvent;
	IL_HANDLE				waitForThreadToStartServicingComPortEvent;

	/**
	 * Used by the user thread to wait until the comPortServiceThread receives
	 * a command response.
	 */
	IL_HANDLE				waitForCommandResponseEvent;

	IL_CRITICAL_SECTION		criticalSection;

	/**
	 * Critical section for communicating over the COM port.
	 */
	IL_CRITICAL_SECTION		critSecForComPort;

	/**
	 * Critical section for accessing the response match fields.
	 */
	IL_CRITICAL_SECTION		critSecForResponseMatchAccess;

	/**
	 * Critical section for accessing the latestAsyncData field.
	 */
	IL_CRITICAL_SECTION		critSecForLatestAsyncDataAccess;

	/**
	 * Signals to the comPortServiceThread if it should continue servicing the
	 * COM port.
	 */
	IL_BOOL					continueServicingComPort;

	/**
	 * This field is used to signal to the comPortServiceThread that it should
	 * be checking to a command response comming from the AHRS_10 device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the AHRS_10. This field should only be accessed in the
	 * functions AHRS_10_shouldCheckForResponse_threadSafe,
	 * AHRS_10_enableResponseChecking_threadSafe, and
	 * AHRS_10_disableResponseChecking_threadSafe.
	 */
	IL_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the AHRS_10 is a match for the command sent.
	 * This field should only be accessed in the functions
	 * AHRS_10_shouldCheckForResponse_threadSafe, AHRS_10_enableResponseChecking_threadSafe,
	 * and AHRS_10_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[IL_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the AHRS_10 device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "AHRS_10_ClbData" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[IL_MAX_RESPONSE_SIZE + 1];

	unsigned char*       	dataBuffer;
	unsigned char*       	nmeaBuffer;
	//unsigned char           dataBuffer[READ_BUFFER_SIZE];
	int 			        cmdResponseSize;

	int 					num_bytes_recive;
	int 					recive_flag;

	AHRS_10_CompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	AHRS_10_NewDataReceivedListener DataListener;

} AHRS_10Internal;

/* Private type definitions. *************************************************/


void* AHRS_10_communicationHandler(void*);

IL_ERROR_CODE  AHRS_10_writeOutCommand(IL_AHRS_10* ahrs, unsigned char cmdToSend[], int cmdSize);


/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	ahrs	Pointer to the IL_AHRS_10 control object.
 *
 * \param[in ]  curResponsePos
 * The size of the packet going to recive
 *
 * \param[out]	responseMatchBuffer
 * \\TODO : add the comment , not sure now
 *
 * \return IL_TRUE if response checking should be performed; Il_FALSE if no
 * checking should be performed.
 */
IL_BOOL AHRS_10_shouldCheckForResponse_threadSafe(IL_AHRS_10* ahrs, char* responseMatchBuffer, int num_bytes_to_recive);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	ahrs	Pointer to the IL_AHRS_10 control object.
 *
 * \param[in]	responseMatch
 * \\ TODO: add the comments here
 */
void AHRS_10_enableResponseChecking_threadSafe(IL_AHRS_10* ahrs, int responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	ahrs 	Pointer to the IL_AHRS_10 control object.
 */
void AHRS_10_disableResponseChecking_threadSafe(IL_AHRS_10* ahrs);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command  and transmits it to the device.
 * The function will then wait until the response is received. The response
 * will be located in the AHRS_10Internal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	ahrs				Pointer to the IL_AHRS_10 control object.
 *
 * \param[in]	responseMatch
 * The size of the data recive & latter need to check the checksum
 *
 * \param[in]	cmdToSend
 * Pointer to the command data to transmit to the device. Should be
 * null-terminated.
 *
 * \return InertialLabs error code.
 */
IL_ERROR_CODE AHRS_10_transaction(IL_AHRS_10* ahrs, unsigned char cmdToSend[], int responseMatch);


/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions  inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int AHRS_10_writeData_threadSafe(IL_AHRS_10* ahrs, unsigned char dataToSend[], unsigned int dataLength);


/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
IL_ERROR_CODE AHRS_10_readData_threadSafe(IL_AHRS_10* ahrs, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a AHRS_10 control object.
 *
 * \param[in]	ahrs	Pointer to the AHRS_10 control object.
 * \return The internal data.
 */
AHRS_10Internal* AHRS_10_getInternalData(IL_AHRS_10* ahrs);

void AHRS_10_processAsyncData(IL_AHRS_10* ahrs, unsigned char buffer[]);

/**
 * \brief  Process and Check the received packaet and transfer in to the internal structure.
 *
 * \param[in]	ahrs	Pointer to the AHRS_10 control object.
 * \param[in]   data buffer
 * \param[in]   Number of bytes recieved from the AHRS_10
 *
 * \return Copy the data buffer to internal data structure variable.
 */
void AHRS_10_processReceivedPacket(IL_AHRS_10* ahrs, unsigned char buffer[], int num_bytes_to_recive);

/**
 * \brief Helper method to set the buffer size in the AHRS_10 control object,according to the command.
 *
 * \param[in]	ahrs	Pointer to the AHRS_10 control object.
 * \return buffer size in  internal data structure.
 */
void AHRS_10_Recive_size(IL_AHRS_10* ahrs);


/* Function definitions. *****************************************************/


IL_ERROR_CODE AHRS_10_connect(IL_AHRS_10* newAHRS_10, const char* portName, int baudrate)
{

	AHRS_10Internal* AHRS_10Int;
	IL_ERROR_CODE errorCode;

	/* Allocate memory. */
	AHRS_10Int = (AHRS_10Internal*)malloc(sizeof(AHRS_10Internal));
	newAHRS_10->internalData = AHRS_10Int;

	newAHRS_10->portName = (char*)malloc(strlen(portName) + 1);
	strcpy(newAHRS_10->portName, portName);
	newAHRS_10->baudRate = baudrate;
	newAHRS_10->isConnected = IL_FALSE;
	AHRS_10Int->continueServicingComPort = IL_TRUE;
	AHRS_10Int->checkForResponse = IL_FALSE;
	AHRS_10Int->timeout = DEFAULT_TIMEOUT_IN_MS;
	AHRS_10Int->DataListener = NULL;
	AHRS_10Int->recive_flag = ILERR_UNKNOWN_ERROR;

	memset(&AHRS_10Int->lastestAsyncData, 0, sizeof(AHRS_10_CompositeData));


	errorCode = inertial_comPort_open(&AHRS_10Int->comPortHandle, portName, baudrate);

	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_criticalSection_initialize(&AHRS_10Int->criticalSection);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRS_10Int->critSecForComPort);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRS_10Int->critSecForResponseMatchAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRS_10Int->critSecForLatestAsyncDataAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_event_create(&AHRS_10Int->waitForThreadToStopServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&AHRS_10Int->waitForCommandResponseEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&AHRS_10Int->waitForThreadToStartServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_thread_startNew(&AHRS_10Int->comPortServiceThreadHandle, &AHRS_10_communicationHandler, newAHRS_10);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	newAHRS_10->isConnected = IL_TRUE;

	errorCode = inertial_event_waitFor(AHRS_10Int->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;


	return ILERR_NO_ERROR;
}


IL_ERROR_CODE AHRS_10_disconnect(IL_AHRS_10* ahrs)
{
	AHRS_10Internal* AHRS_10Int;

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	AHRS_10Int->continueServicingComPort = IL_FALSE;

	inertial_event_waitFor(AHRS_10Int->waitForThreadToStopServicingComPortEvent, -1);
	AHRS_10_Stop(ahrs);
	//sleep(3);

	inertial_comPort_close(AHRS_10Int->comPortHandle);

	inertial_criticalSection_dispose(&AHRS_10Int->criticalSection);
	inertial_criticalSection_dispose(&AHRS_10Int->critSecForComPort);
	inertial_criticalSection_dispose(&AHRS_10Int->critSecForResponseMatchAccess);
	inertial_criticalSection_dispose(&AHRS_10Int->critSecForLatestAsyncDataAccess);

	/* Free the memory associated with the AHRS_10 structure. */
	//free(&AHRS_10Int->waitForThreadToStopServicingComPortEvent);
	//free(&AHRS_10Int->waitForCommandResponseEvent);
	//free(&AHRS_10Int->waitForThreadToStartServicingComPortEvent);
	free(ahrs->internalData);
	ahrs->isConnected = IL_FALSE;
#if defined(WIN32) || defined(WIN64) 
	Sleep(3000);
#else
	//sleep(3);
#endif 
	return ILERR_NO_ERROR;
}


int AHRS_10_writeData_threadSafe(IL_AHRS_10* ahrs, unsigned char dataToSend[], unsigned int dataLength)
{
	int errorCode;
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

#if IL_AHRS_10_DBG
	printf("AHRS_10_writeData_threadSafe : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n", dataToSend[0], dataToSend[1], dataToSend[2],
		dataToSend[3], dataToSend[4], dataToSend[5],
		dataToSend[6], dataToSend[7], dataToSend[8]);

#endif 
	inertial_criticalSection_enter(&AHRS_10Int->critSecForComPort);
	errorCode = inertial_comPort_writeData(AHRS_10Int->comPortHandle, dataToSend, dataLength);
	inertial_criticalSection_leave(&AHRS_10Int->critSecForComPort);

	//printf("after read");

	return errorCode;
}

IL_ERROR_CODE AHRS_10_readData_threadSafe(IL_AHRS_10* ahrs, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	IL_ERROR_CODE errorCode;
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRS_10Int->critSecForComPort);
	errorCode = inertial_comPort_readData(AHRS_10Int->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	inertial_criticalSection_leave(&AHRS_10Int->critSecForComPort);

	if (ahrs->mode)
	{

		if ((ahrs->cmd_flag == IL_AHRS_10_CLB_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_AHRS_10_CLB_DATA_CMD_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_10_CLB_HR_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_AHRS_10_CLB_DATA_HR_DATA_CMD_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_10_QUATERNION_RECEIVE && *numOfBytesActuallyRead == (int)IL_AHRS_10_QUATERNION_CMD_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_10_GET_DEV_INFO_RECEIVE && *numOfBytesActuallyRead == (int)IL_AHRS_10_GET_DEV_INFO_CMD_RECEIVE_SIZE)) 
		{
			return ILERR_NO_ERROR;
		}
		else if (ahrs->cmd_flag == IL_AHRS_10_STOP_CMD && *numOfBytesActuallyRead == IL_AHRS_10_STOP_CMD_RECEIVE_SIZE)
		{

			return ILERR_NO_ERROR;
		}
		else
		{
			//printf(" error data recived :%d" , *numOfBytesActuallyRead);
			return ILERR_RECIVE_SIZE_ERROR;

		}
	}

	return errorCode;

}

IL_ERROR_CODE AHRS_10_writeOutCommand(IL_AHRS_10* ahrs, unsigned char cmdToSend[], int cmdSize)
{
	//char packetTail[] = {0xFF ,0xFF};
	 //printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  %d \n", cmdToSend[0] ,cmdToSend[1] ,cmdToSend[2],
	 //																				cmdToSend[3] ,cmdToSend[4],cmdToSend[5],
	 //																				cmdToSend[6] ,cmdToSend[7],cmdToSend[8], cmdSize);

	 //printf("size of : %d \n ", strlen(cmdToSend));
	AHRS_10_writeData_threadSafe(ahrs, cmdToSend, cmdSize);

	//
	//AHRS_10_writeData_threadSafe(ahrs,packetTail, strlen(packetTail));

	return ILERR_NO_ERROR;
}


void* AHRS_10_communicationHandler(void* AHRS_10obj)
{

	IL_AHRS_10* ahrs;
	AHRS_10Internal* AHRS_10Int;
	unsigned char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	unsigned char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	IL_BOOL haveFoundStartOfCommand = IL_FALSE;

	memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);

	ahrs = (IL_AHRS_10*)AHRS_10obj;
	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_event_signal(AHRS_10Int->waitForThreadToStartServicingComPortEvent);

	while (AHRS_10Int->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		AHRS_10_readData_threadSafe(ahrs, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);

// #if IL_AHRS_10_DBG
// 		printf("numOfBytesRead %d : \n", numOfBytesRead);

// #endif

		if (numOfBytesRead == 0)
		{
			// There was no data. Sleep for a short amount of time before continuing.
			inertial_sleepInMs(NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_NOT_RECEIVING_ON_COM_PORT);
			continue;
		}
		else if (numOfBytesRead > 0 && numOfBytesRead < AHRS_10Int->num_bytes_recive)
		{
			printf("numOfBytesRead %d : \n", numOfBytesRead);
#if IL_AHRS_10_DBG
			printf("complete package not recived , so wrong value in the packet \n");

#endif 	
			memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);
			continue;		
		}
		else if(numOfBytesRead > 0 && numOfBytesRead == AHRS_10Int->num_bytes_recive)
		{
#if IL_AHRS_10_DBG
			for (int i = 0; i < numOfBytesRead; i++)
			{
				printf("0x%02x  ", readBuffer[i]);
			}
			printf("\n");
			printf("numOfBytesRead %d : \n", numOfBytesRead);
			printf("number of bytes to recive : %d \n", AHRS_10Int->num_bytes_recive);
			printf(" cmd_flag : %d \n", ahrs->cmd_flag);
#endif 

			switch (ahrs->cmd_flag)
			{
			case IL_AHRS_10_STOP_CMD:
#if IL_AHRS_10_DBG
				printf("IL_STOP_CMD \n");
#endif
				break;

			case IL_AHRS_10_ONREQUEST_CMD:
#if IL_AHRS_10_DBG
				printf("IL_SET_ONREQUEST_CMD \n");
#endif
				break;

			case IL_AHRS_10_CLB_DATA_RECEIVE:
			case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
			case IL_AHRS_10_QUATERNION_RECEIVE:
			case IL_AHRS_10_GET_DEV_INFO_RECEIVE:
			{
				if (ahrs->mode)
				{

					//inertial_criticalSection_enter(&AHRS_10Int->critSecForResponseMatchAccess);
					AHRS_10_processReceivedPacket(ahrs, readBuffer, AHRS_10Int->num_bytes_recive);
					//inertial_criticalSection_leave(&AHRS_10Int->critSecForResponseMatchAccess);
				}
				else
				{
					for (; curResponsePos < numOfBytesRead; curResponsePos++) {

						if (responseBuilderBufferPos > RESPONSE_BUILDER_BUFFER_SIZE) {
							/* We are about to overfill our buffer. Let's just reinitialize the buffer. */
							printf(" We are about to overfill our buffer \n");
							haveFoundStartOfCommand = IL_FALSE;
							responseBuilderBufferPos = 0;
						}
						/* See if we have even found the start of a response. */
						if (((int)readBuffer[curResponsePos] == 170) && ((int)readBuffer[curResponsePos + 1] == 85)) {						
#if IL_AHRS_10_DBG
							printf(" found the start byte \n");
							for (int i = 0; i < numOfBytesRead; i++)
							{
								printf("0x%02x  ", readBuffer[i]);
							}
							printf("\n");
#endif
							/* Alright, we have found the start of a command. */
							//free(AHRS_10Int->dataBuffer);
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

						}
						if (((int)readBuffer[curResponsePos] == 170) && ((int)readBuffer[curResponsePos + 1] == 68) && ((int)readBuffer[curResponsePos + 2] == 18) ) {
#if IL_AHRS_10_DBG
							printf(" found the start byte \n");
							for (int i = 0; i < numOfBytesRead; i++)
							{
								printf("0x%02x  ", readBuffer[i]);
							}
							printf("\n");
#endif
							/* Alright, we have found the start of a command. */
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

						}

						if (haveFoundStartOfCommand) {

							responseBuilderBuffer[responseBuilderBufferPos] = readBuffer[curResponsePos];
							responseBuilderBufferPos++;
							if (responseBuilderBufferPos == AHRS_10Int->num_bytes_recive)
							{
								haveFoundStartOfCommand = IL_FALSE;
							}
						}

						if (responseBuilderBufferPos == AHRS_10Int->num_bytes_recive)
						{
							//inertial_criticalSection_enter(&AHRS_10Int->critSecForResponseMatchAccess);
							AHRS_10_processReceivedPacket(ahrs, responseBuilderBuffer, AHRS_10Int->num_bytes_recive);
							responseBuilderBufferPos = 0;
							haveFoundStartOfCommand = IL_FALSE;
							//inertial_criticalSection_leave(&AHRS_10Int->critSecForResponseMatchAccess);


						}
						else {
							/* We received data but we have not found the starting point of a command. */

						}
					}
				}

				break;
			}
			case IL_READ_AHRS_10_PAR_RECEIVE:
			{
				AHRS_10_processReceivedPacket(ahrs, readBuffer, AHRS_10Int->num_bytes_recive);
			}
			case IL_AHRS_10_NMEA_RECEIVE:
			{
				if (ahrs->mode)
				{
					for (int i = 0; i < numOfBytesRead; i++)
					{
						printf("%c", readBuffer[i]);
					}
				}
				else
				{
					
				}
			}
			default:
#if IL_AHRS_10_DBG
				printf("No CMD flag Set !!!! \n");
#endif
				break;
			}

		}
	}
	inertial_event_signal(AHRS_10Int->waitForThreadToStopServicingComPortEvent);

	return IL_NULL;

}


IL_BOOL AHRS_10_verifyConnectivity(IL_AHRS_10* ahrs)
{
	//TODO 
	return true;
}
void textFromHexString(char *hex, char *result)
{
	char text[25] = { 0 };
	int tc = 0;

	for (int k = 0; k < strlen(hex); k++)
	{
		if (k % 2 != 0)
		{
			char temp[3];
			sprintf(temp, "%c%c", hex[k - 1], hex[k]);
			int number = (int)strtol(temp, NULL, 16);
			text[tc] = (char)(number);

			tc++;

		}
	}
	strcpy(result, text);

}

IL_ERROR_CODE AHRS_10_GetDevInfoParameters(IL_AHRS_10* ahrs , AHRS_10_GetDevInfoData* data)
{


}

IL_ERROR_CODE AHRS_10_ReadInternalParameters(IL_AHRS_10* ahrs, AHRS_10_SetInternalData* data)
{

	int i = 6;
	AHRS_10Internal* AHRS_10Int;
	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	//printf("0x%04x", *(uint16_t*)(&(AHRS_10Int->dataBuffer[i+0])));

	data->Data_rate = (double)(*(uint16_t*)(&(AHRS_10Int->dataBuffer[i + 0])));

	data->Initial_Alignment_Time = (double)(*(uint16_t*)(&(AHRS_10Int->dataBuffer[i + 2])));

	data->Magnetic_Declination = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) ;

	data->Latitude = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 8]))) ;

	data->Longitude = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 12]))) ;

	data->Altitude = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 16]))) ;

	data->Date = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 20])));

	// data->Year = (int)(*(int8_t*)(&(AHRS_10Int->dataBuffer[i + 20])));

	// data->Month = (int)(*(int8_t*)(&(AHRS_10Int->dataBuffer[i + 21])));

	// data->Day = (int)(*(int8_t*)(&(AHRS_10Int->dataBuffer[i + 22])));

	data->Alignment_Angle_A1 = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 24]))) ;

	data->Alignment_Angle_A2 = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 28]))) ;

	data->Alignment_Angle_A3 = (double)(*(float_t*)(&(AHRS_10Int->dataBuffer[i + 32]))) ;

	//data->Device_Id = (char *)(*(int64_t*)(&(AHRS_10Int->dataBuffer[i + 36])));

	//printf("\nDate:  %d - %d - %d\n", data->Year, data->Month, data->Day);

	printf("\nData Rate: %f\n", data->Data_rate);

	printf(
		"\n Initial Alignment Time:  %f\n",
		data->Initial_Alignment_Time);

	printf(
		"\nMagnetic declination, Mdec : %f\n",
		data->Magnetic_Declination);

	printf(
		"\n Latitude : %f\n"
		"  Longitude : %f\n"
		"  Altitude  : %f\n",
		data->Latitude,
		data->Longitude,
		data->Altitude);


	printf(
		"\n Alignment_Angle_A1 : %f\n"
		"  Alignment_Angle_A2 : %f\n"
		"  Alignment_Angle_A3  : %f\n",
		data->Alignment_Angle_A1,
		data->Alignment_Angle_A2,
		data->Alignment_Angle_A3);

	return ILERR_NO_ERROR;

}


AHRS_10Internal* AHRS_10_getInternalData(IL_AHRS_10* ahrs)
{
	return (AHRS_10Internal*)ahrs->internalData;
}

IL_ERROR_CODE AHRS_10_transaction(IL_AHRS_10* ahrs, unsigned char *cmdToSend, int responseMatch)
{
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);


	AHRS_10_enableResponseChecking_threadSafe(ahrs, responseMatch);

	AHRS_10_writeData_threadSafe(ahrs, cmdToSend, sizeof(cmdToSend));

	return inertial_event_waitFor(AHRS_10Int->waitForCommandResponseEvent, AHRS_10Int->timeout);
}

void AHRS_10_enableResponseChecking_threadSafe(IL_AHRS_10* ahrs, int responseMatch)
{
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRS_10Int->critSecForResponseMatchAccess);

	AHRS_10Int->checkForResponse = IL_TRUE;
	AHRS_10Int->cmdResponseSize = responseMatch;

	inertial_criticalSection_leave(&AHRS_10Int->critSecForResponseMatchAccess);
}


void AHRS_10_processReceivedPacket(IL_AHRS_10* ahrs, unsigned char buffer[], int num_bytes_to_recive)
{
	AHRS_10Internal* AHRS_10Int;


	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRS_10Int->critSecForLatestAsyncDataAccess);
	AHRS_10Int->dataBuffer = buffer;
	AHRS_10Int->recive_flag = ILERR_DATA_IN_BUFFER;
	inertial_criticalSection_leave(&AHRS_10Int->critSecForLatestAsyncDataAccess);

#if  IL_RAW_DATA
	for (int i = 0; i < AHRS_10Int->num_bytes_recive; i++)
	{
		printf("0x%02x ", AHRS_10Int->dataBuffer[i]);
	}
	printf("\n");

#endif

#if 0
	/* See if we should be checking for a command response. */
	if (AHRS_10_shouldCheckForResponse_threadSafe(ahrs, buffer, num_bytes_to_recive)) {

		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		//if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			 //AHRS_10_disableResponseChecking_threadSafe(ahrs);

			 /* The line below should be thread-safe since the user thread should be
			  * blocked until we signal that we have received the response. */


			  /* Signal to the user thread we have received a response. */
		inertial_event_signal(AHRS_10Int->waitForCommandResponseEvent);
		//}
	}
#endif 

}


IL_ERROR_CODE AHRS_10_YPR(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i =  6;
	AHRS_10Internal* AHRS_10Int;

	printf("inside AHRS_10_YPR\n");

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_10_CLB_DATA_RECEIVE:
	case IL_AHRS_10_QUATERNION_RECEIVE:
	{
		data->ypr.yaw = (double)(*(uint16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / 100;
		data->ypr.pitch = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) / 100;
		data->ypr.roll = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / 100;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
	{
		data->ypr.yaw = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / 1000;
		data->ypr.pitch = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / 1000;
		data->ypr.roll = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 8]))) / 1000;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		
		printf("YPR (Heading , Pitch , Roll)  not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode;

}


IL_ERROR_CODE AHRS_10_getGyroAccMag(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int kg = 50;
	int ka = 4000;
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_10_CLB_DATA_RECEIVE:
	{

	
		i = 12;

		data->gyro.c0 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / kg;
		data->gyro.c1 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) / kg;
		data->gyro.c2 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / kg;


		i = i + 6;

		data->acceleration.c0 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / ka;
		data->acceleration.c1 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) / ka;
		data->acceleration.c2 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / ka;

		i = i + 6;

		data->magnetic.c0 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) * 10;

		errorCode = ILERR_NO_ERROR;

		break;
	}
	case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
	{
		
		i = 6 ;
		data->gyro.c0 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / 100000;
		data->gyro.c1 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / 100000;
		data->gyro.c2 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 8]))) / 100000;

		i = 18;

		data->acceleration.c0 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / 1000000;
		data->acceleration.c1 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / 1000000;
		data->acceleration.c2 = (double)(*(int32_t*)(&(AHRS_10Int->dataBuffer[i + 8]))) / 1000000;


		i = 30;

		data->magnetic.c0 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) * 10;

		errorCode = ILERR_NO_ERROR;
		break;
	
	}
	default:
	{
		printf("Gyro , Accleration , Magnetic  not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode ;
}


IL_ERROR_CODE AHRS_10_getQuaternionData(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int quat_conv = 10000;
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;


	switch (ahrs->cmd_flag)
	{

	case IL_AHRS_10_QUATERNION_RECEIVE:
	{
		i = 12;

		data->quaternion.Lk0 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / quat_conv;
		data->quaternion.Lk1 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) / quat_conv;
		data->quaternion.Lk2 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 4]))) / quat_conv;
		data->quaternion.Lk3 = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 6]))) / quat_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		printf("Quaternion are not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode;

}

IL_ERROR_CODE AHRS_10_getSensorData(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_10_CLB_DATA_RECEIVE:
	case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
	case IL_AHRS_10_QUATERNION_RECEIVE:
	{

		i = (ahrs->cmd_flag == IL_AHRS_10_CLB_HR_DATA_RECEIVE) ? 54 : 36;

		data->Vinp = (double)(*(uint16_t*)(&(AHRS_10Int->dataBuffer[i + 0]))) / 100;
		data->Temper = (double)(*(int16_t*)(&(AHRS_10Int->dataBuffer[i + 2]))) / 10;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		printf("Temp ,  Input Voltage data not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode ;

}

IL_ERROR_CODE AHRS_10_SetMode(IL_AHRS_10* ahrs, int mode)
{
	ahrs->mode = mode;
	unsigned int syncDataOutputType = 100;
	IL_BOOL waitForResponse = 1;
	if (ahrs->mode)
	{
		ahrs->cmd_flag = IL_AHRS_10_ONREQUEST_CMD;
		AHRS_10_SetOnRequestMode(ahrs, syncDataOutputType, waitForResponse);
		return ILERR_NO_ERROR;
	}
	else
	{
		printf("now you can call datamode functions you want to recive from the AHRS_10 ");
#if defined(WIN32) || defined(WIN64) 
		Sleep(2000);
#else
		sleep(2);
#endif 


		return ILERR_NO_ERROR;
	}
}


IL_ERROR_CODE AHRS_10_Stop(IL_AHRS_10* ahrs)
{
	int errorCode;
	printf("Stopping the AHRS_10 device \n");
	unsigned char stop_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xFE ,0x05, 0x01 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("size of stop %d \n" , sizeof(stop_payload));
	ahrs->cmd_flag = IL_AHRS_10_STOP_CMD;
	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, stop_payload, sizeof(stop_payload));
	return errorCode;
}

IL_ERROR_CODE AHRS_10_SetOnRequestMode(IL_AHRS_10* ahrs, unsigned int syncDataOutputType, IL_BOOL waitForResponse)
{

	int errorCode;
	unsigned char  setonrequest_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xC1 ,0xC8 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_SetOnRequestMode\n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, setonrequest_payload, sizeof(setonrequest_payload));

	//printf("inside AHRS_10_SetOnRequestMode done\n");
	return errorCode;
}

IL_ERROR_CODE AHRS_10_ClbData_Receive(IL_AHRS_10* ahrs)
{
	int errorCode;
	unsigned char  ClbData_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x83 ,0x8A ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_ClbData_Receive \n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, ClbData_payload, sizeof(ClbData_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_10_ClbHRData_Receive(IL_AHRS_10* ahrs)
{
	int errorCode;
	unsigned char  ClbHRData_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x81 ,0x88 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_ClbHRData_Receive \n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, ClbHRData_payload, sizeof(ClbHRData_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_10_QuatData_Receive(IL_AHRS_10* ahrs)
{
	int errorCode;
	unsigned char  QuatData_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x82 ,0x89 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_QuatData_Receive \n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, QuatData_payload, sizeof(QuatData_payload));

	return errorCode;
}


IL_ERROR_CODE AHRS_10_NMEA_Receive(IL_AHRS_10* ahrs) 
{
	int errorCode;
	unsigned char  NMEA_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x88 ,0x8F ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_NMEA_Receive \n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, NMEA_payload, sizeof(NMEA_payload));

	return errorCode;
}


IL_ERROR_CODE ReadAHRS10par(IL_AHRS_10* ahrs)
{
	int errorCode;
	unsigned char  ReadAHRS_10par_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x41 ,0x48 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside ReadAHRS_10par \n");

	ahrs->cmd_flag = IL_READ_AHRS_10_PAR_RECEIVE;

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, ReadAHRS_10par_payload, sizeof(ReadAHRS_10par_payload));


	return errorCode;
}

IL_ERROR_CODE LoadAHRS10par(IL_AHRS_10* ahrs)
{
	int errorCode;

	unsigned char  LoadAHRS_10par_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x40 ,0x47 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	printf("inside LoadAHRS_10par \n");

	ahrs->cmd_flag = IL_LOAD_AHRS_10_PAR_RECEIVE;

	//AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, LoadAHRS_10par_payload, sizeof(LoadAHRS_10par_payload));

	return errorCode;
}


IL_ERROR_CODE AHRS_10_GetDevInfo(IL_AHRS_10* ahrs)
{
	int errorCode;

	unsigned char  GetDevInfo_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x12 ,0x19 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_10_GetDevInfo \n");

	AHRS_10_Recive_size(ahrs);
	errorCode = AHRS_10_writeOutCommand(ahrs, GetDevInfo_payload, sizeof(GetDevInfo_payload));

	return errorCode;
}


void AHRS_10_Recive_size(IL_AHRS_10* ahrs)
{
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);
#if IL_AHRS_10_DBG
	printf("now ahrs->cmd_flag : %d", ahrs->cmd_flag);
#endif
	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_10_STOP_CMD:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_STOP_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_ONREQUEST_CMD:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_SET_ONREQUEST_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_CLB_DATA_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_CLB_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_CLB_DATA_HR_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_QUATERNION_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_QUATERNION_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_NMEA_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_NMEA_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_10_GET_DEV_INFO_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_AHRS_10_GET_DEV_INFO_CMD_RECEIVE_SIZE;
		break;
	case IL_READ_AHRS_10_PAR_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_READ_AHRS_10_PAR_CMD_RECEIVE_SIZE;
		break;
	case IL_LOAD_AHRS_10_PAR_RECEIVE:
		AHRS_10Int->num_bytes_recive = IL_LOAD_AHRS_10_PAR_CMD_RECEIVE_SIZE;
		break;
	default:
		break;
	}
}

IL_BOOL AHRS_10_shouldCheckForResponse_threadSafe(IL_AHRS_10* ahrs, char* responseMatchBuffer, int num_bytes_to_recive)
{
	AHRS_10Internal* AHRS_10Int;
	IL_BOOL shouldCheckResponse;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRS_10Int->critSecForResponseMatchAccess);
	shouldCheckResponse = 1; // to avoid warning now
	inertial_criticalSection_leave(&AHRS_10Int->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}


void AHRS_10_disableResponseChecking_threadSafe(IL_AHRS_10* ahrs)
{
	AHRS_10Internal* AHRS_10Int;

	AHRS_10Int = AHRS_10_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRS_10Int->critSecForResponseMatchAccess);

	AHRS_10Int->checkForResponse = IL_FALSE;
	AHRS_10Int->cmdResponseSize = 0;
	AHRS_10Int->cmdResponseMatchBuffer[0] = 0;

	inertial_criticalSection_leave(&AHRS_10Int->critSecForResponseMatchAccess);
}


IL_ERROR_CODE AHRS_10_registerDataReceivedListener(IL_AHRS_10* ahrs, AHRS_10_NewDataReceivedListener listener)
{
	AHRS_10Internal* AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->DataListener != NULL)
		return ILERR_UNKNOWN_ERROR;

	AHRS_10Int->DataListener = listener;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE AHRS_10_unregisterDataReceivedListener(IL_AHRS_10* ahrs, AHRS_10_NewDataReceivedListener listener)
{
	AHRS_10Internal* AHRS_10Int = AHRS_10_getInternalData(ahrs);

	if (AHRS_10Int->DataListener == NULL)
		return ILERR_UNKNOWN_ERROR;

	if (AHRS_10Int->DataListener != listener)
		return ILERR_UNKNOWN_ERROR;

	AHRS_10Int->DataListener = NULL;

	return ILERR_NO_ERROR;
}

unsigned char AHRS_10_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char)cmdToCheck[i];

	return xorVal;
}

void AHRS_10_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = AHRS_10_checksum_compute(cmdToCheck);
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

void AHRS_10_processAsyncData(IL_AHRS_10* ahrs, unsigned char buffer[])
{
	AHRS_10_CompositeData data;
	AHRS_10Internal* AHRS_10Int = AHRS_10_getInternalData(ahrs);

	memset(&data, 0, sizeof(AHRS_10_CompositeData));


	/* We had an async data packet and need to move it to AHRS_10Int->lastestAsyncData. */
	inertial_criticalSection_enter(&AHRS_10Int->critSecForLatestAsyncDataAccess);
	memcpy(&AHRS_10Int->lastestAsyncData, &data, sizeof(AHRS_10_CompositeData));
	inertial_criticalSection_leave(&AHRS_10Int->critSecForLatestAsyncDataAccess);

	if (AHRS_10Int->DataListener != NULL)
		AHRS_10Int->DataListener(ahrs, &AHRS_10Int->lastestAsyncData);


}




/** \endcond */