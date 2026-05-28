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
#include "InertialLabs_AHRS.h"
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
	 * be checking to a command response comming from the AHRS device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the AHRS. This field should only be accessed in the
	 * functions AHRS_shouldCheckForResponse_threadSafe,
	 * AHRS_enableResponseChecking_threadSafe, and
	 * AHRS_disableResponseChecking_threadSafe.
	 */
	IL_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the AHRS is a match for the command sent.
	 * This field should only be accessed in the functions
	 * AHRS_shouldCheckForResponse_threadSafe, AHRS_enableResponseChecking_threadSafe,
	 * and AHRS_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[IL_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the AHRS device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "AHRS_ClbData" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[IL_MAX_RESPONSE_SIZE + 1];

	unsigned char*       	dataBuffer;
	//unsigned char           dataBuffer[READ_BUFFER_SIZE];
	int 			        cmdResponseSize;

	int 					num_bytes_recive;
	int 					recive_flag;

	AHRSCompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	AHRS_NewDataReceivedListener DataListener;

} AHRSInternal;

/* Private type definitions. *************************************************/


void* AHRS_communicationHandler(void*);

IL_ERROR_CODE  AHRS_writeOutCommand(IL_AHRS* ahrs, unsigned char cmdToSend[], int cmdSize);


/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	ahrs	Pointer to the IL_AHRS control object.
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
IL_BOOL AHRS_shouldCheckForResponse_threadSafe(IL_AHRS* ahrs, char* responseMatchBuffer, int num_bytes_to_recive);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	ahrs	Pointer to the IL_AHRS control object.
 *
 * \param[in]	responseMatch
 * \\ TODO: add the comments here
 */
void AHRS_enableResponseChecking_threadSafe(IL_AHRS* ahrs, int responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	ahrs 	Pointer to the IL_AHRS control object.
 */
void AHRS_disableResponseChecking_threadSafe(IL_AHRS* ahrs);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command  and transmits it to the device.
 * The function will then wait until the response is received. The response
 * will be located in the AHRSInternal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	ahrs				Pointer to the IL_AHRS control object.
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
IL_ERROR_CODE AHRS_transaction(IL_AHRS* ahrs, unsigned char cmdToSend[], int responseMatch);


/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions  inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int AHRS_writeData_threadSafe(IL_AHRS* ahrs, unsigned char dataToSend[], unsigned int dataLength);


/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
IL_ERROR_CODE AHRS_readData_threadSafe(IL_AHRS* ahrs, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a AHRS control object.
 *
 * \param[in]	ahrs	Pointer to the AHRS control object.
 * \return The internal data.
 */
AHRSInternal* AHRS_getInternalData(IL_AHRS* ahrs);

void AHRS_processAsyncData(IL_AHRS* ahrs, unsigned char buffer[]);

/**
 * \brief  Process and Check the received packaet and transfer in to the internal structure.
 *
 * \param[in]	ahrs	Pointer to the AHRS control object.
 * \param[in]   data buffer
 * \param[in]   Number of bytes recieved from the AHRS
 *
 * \return Copy the data buffer to internal data structure variable.
 */
void AHRS_processReceivedPacket(IL_AHRS* ahrs, unsigned char buffer[], int num_bytes_to_recive);

/**
 * \brief Helper method to set the buffer size in the AHRS control object,according to the command.
 *
 * \param[in]	ahrs	Pointer to the AHRS control object.
 * \return buffer size in  internal data structure.
 */
void AHRS_Recive_size(IL_AHRS* ahrs);


/* Function definitions. *****************************************************/


IL_ERROR_CODE AHRS_connect(IL_AHRS* newAHRS, const char* portName, int baudrate)
{

	AHRSInternal* AHRSInt;
	IL_ERROR_CODE errorCode;

	/* Allocate memory. */
	AHRSInt = (AHRSInternal*)malloc(sizeof(AHRSInternal));
	newAHRS->internalData = AHRSInt;

	newAHRS->portName = (char*)malloc(strlen(portName) + 1);
	strcpy(newAHRS->portName, portName);
	newAHRS->baudRate = baudrate;
	newAHRS->isConnected = IL_FALSE;
	AHRSInt->continueServicingComPort = IL_TRUE;
	AHRSInt->checkForResponse = IL_FALSE;
	AHRSInt->timeout = DEFAULT_TIMEOUT_IN_MS;
	AHRSInt->DataListener = NULL;
	AHRSInt->recive_flag = ILERR_UNKNOWN_ERROR;

	memset(&AHRSInt->lastestAsyncData, 0, sizeof(AHRSCompositeData));


	errorCode = inertial_comPort_open(&AHRSInt->comPortHandle, portName, baudrate);

	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_criticalSection_initialize(&AHRSInt->criticalSection);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRSInt->critSecForComPort);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRSInt->critSecForResponseMatchAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&AHRSInt->critSecForLatestAsyncDataAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_event_create(&AHRSInt->waitForThreadToStopServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&AHRSInt->waitForCommandResponseEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&AHRSInt->waitForThreadToStartServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_thread_startNew(&AHRSInt->comPortServiceThreadHandle, &AHRS_communicationHandler, newAHRS);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	newAHRS->isConnected = IL_TRUE;

	errorCode = inertial_event_waitFor(AHRSInt->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;


	return ILERR_NO_ERROR;
}


IL_ERROR_CODE AHRS_disconnect(IL_AHRS* ahrs)
{
	AHRSInternal* AHRSInt;

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	AHRSInt = AHRS_getInternalData(ahrs);

	AHRSInt->continueServicingComPort = IL_FALSE;

	inertial_event_waitFor(AHRSInt->waitForThreadToStopServicingComPortEvent, -1);
	AHRS_Stop(ahrs);
	//sleep(3);

	inertial_comPort_close(AHRSInt->comPortHandle);

	inertial_criticalSection_dispose(&AHRSInt->criticalSection);
	inertial_criticalSection_dispose(&AHRSInt->critSecForComPort);
	inertial_criticalSection_dispose(&AHRSInt->critSecForResponseMatchAccess);
	inertial_criticalSection_dispose(&AHRSInt->critSecForLatestAsyncDataAccess);


	/* Free the memory associated with the AHRS structure. */
	//free(&AHRSInt->waitForThreadToStopServicingComPortEvent);
	//free(&AHRSInt->waitForCommandResponseEvent);
	//free(&AHRSInt->waitForThreadToStartServicingComPortEvent);
	free(ahrs->internalData);

	ahrs->isConnected = IL_FALSE;
#if defined(WIN32) || defined(WIN64) 
	Sleep(3000);
#else
	sleep(3);
#endif 

	return ILERR_NO_ERROR;
}


int AHRS_writeData_threadSafe(IL_AHRS* ahrs, unsigned char dataToSend[], unsigned int dataLength)
{
	int errorCode;
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

#if IL_AHRS_DBG
	printf("AHRS_writeData_threadSafe : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n", dataToSend[0], dataToSend[1], dataToSend[2],
		dataToSend[3], dataToSend[4], dataToSend[5],
		dataToSend[6], dataToSend[7], dataToSend[8]);

#endif 
	inertial_criticalSection_enter(&AHRSInt->critSecForComPort);
	errorCode = inertial_comPort_writeData(AHRSInt->comPortHandle, dataToSend, dataLength);
	inertial_criticalSection_leave(&AHRSInt->critSecForComPort);

	//printf("after read");

	return errorCode;
}

IL_ERROR_CODE AHRS_readData_threadSafe(IL_AHRS* ahrs, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	IL_ERROR_CODE errorCode;
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRSInt->critSecForComPort);
	errorCode = inertial_comPort_readData(AHRSInt->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	inertial_criticalSection_leave(&AHRSInt->critSecForComPort);

	if (ahrs->mode)
	{

		if ((ahrs->cmd_flag == IL_AHRS_CONT_1_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_FULL_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_CONT_2_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_ORIENTATION_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_CONT_3_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_ORIENTATION_SENSOR_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_REQ_1_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_FULL_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_REQ_2_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_ORIENTATION_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_REQ_3_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_ORIENTATION_SENSOR_OUTPUT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_GET_BIT_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_GET_BIT_RECEIVE_SIZE) ||
			(ahrs->cmd_flag == IL_AHRS_GET_VER_FIRMWARE_CMD && *numOfBytesActuallyRead == (int)IL_AHRS_GET_FIRMWARE_VER_RECEIVE_SIZE) ) 
		{
			return ILERR_NO_ERROR;
		}
		else if (ahrs->cmd_flag == IL_AHRS_STOP_CMD && *numOfBytesActuallyRead == IL_AHRS_STOP_CMD_RECEIVE_SIZE)
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

IL_ERROR_CODE AHRS_writeOutCommand(IL_AHRS* ahrs, unsigned char cmdToSend[], int cmdSize)
{
	//char packetTail[] = {0xFF ,0xFF};
	 //printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  %d \n", cmdToSend[0] ,cmdToSend[1] ,cmdToSend[2],
	 //																				cmdToSend[3] ,cmdToSend[4],cmdToSend[5],
	 //																				cmdToSend[6] ,cmdToSend[7],cmdToSend[8], cmdSize);

	 //printf("size of : %d \n ", strlen(cmdToSend));
	AHRS_writeData_threadSafe(ahrs, cmdToSend, cmdSize);

	//
	//AHRS_writeData_threadSafe(ahrs,packetTail, strlen(packetTail));

	return ILERR_NO_ERROR;
}


void* AHRS_communicationHandler(void* AHRSobj)
{

	IL_AHRS* ahrs;
	AHRSInternal* AHRSInt;
	unsigned char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	unsigned char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	IL_BOOL haveFoundStartOfCommand = IL_FALSE;

	memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);

	ahrs = (IL_AHRS*)AHRSobj;
	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_event_signal(AHRSInt->waitForThreadToStartServicingComPortEvent);

	while (AHRSInt->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		AHRS_readData_threadSafe(ahrs, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);


		if (numOfBytesRead == 0)
		{
			// There was no data. Sleep for a short amount of time before continuing.
			inertial_sleepInMs(NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_NOT_RECEIVING_ON_COM_PORT);
			continue;
		}
		else
		{
#if IL_AHRS_DBG
			for (int i = 0; i < numOfBytesRead; i++)
			{
				printf("0x%02x  ", readBuffer[i]);
			}
			printf("\n");
			printf("numOfBytesRead %d : \n", numOfBytesRead);
			printf("number of bytes to recive : %d \n", AHRSInt->num_bytes_recive);
			printf(" cmd_flag : %d \n", ahrs->cmd_flag);
#endif 

			switch (ahrs->cmd_flag)
			{
			case IL_AHRS_STOP_CMD:
#if IL_AHRS_DBG
				printf("IL_STOP_CMD \n");
#endif
				break;

			case IL_AHRS_CONT_1_CMD:
			case IL_AHRS_CONT_2_CMD:
			case IL_AHRS_CONT_3_CMD:
			case IL_AHRS_REQ_1_CMD:
			case IL_AHRS_REQ_2_CMD:
			case IL_AHRS_REQ_3_CMD:
			{
				if (ahrs->mode)
				{

					//inertial_criticalSection_enter(&AHRSInt->critSecForResponseMatchAccess);
					AHRS_processReceivedPacket(ahrs, readBuffer, AHRSInt->num_bytes_recive);
					//inertial_criticalSection_leave(&AHRSInt->critSecForResponseMatchAccess);
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
#if IL_AHRS_DBG
							printf(" found the start byte \n");
							for (int i = 0; i < numOfBytesRead; i++)
							{
								printf("0x%02x  ", readBuffer[i]);
							}
							printf("\n");
#endif
							/* Alright, we have found the start of a command. */
							//free(AHRSInt->dataBuffer);
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

						}
						if (((int)readBuffer[curResponsePos] == 170) && ((int)readBuffer[curResponsePos + 1] == 68) && ((int)readBuffer[curResponsePos + 2] == 18) ) {
#if IL_AHRS_DBG
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
							if (responseBuilderBufferPos == AHRSInt->num_bytes_recive)
							{
								haveFoundStartOfCommand = IL_FALSE;
							}
						}

						if (responseBuilderBufferPos == AHRSInt->num_bytes_recive)
						{
							//inertial_criticalSection_enter(&AHRSInt->critSecForResponseMatchAccess);
							AHRS_processReceivedPacket(ahrs, responseBuilderBuffer, AHRSInt->num_bytes_recive);
							responseBuilderBufferPos = 0;
							haveFoundStartOfCommand = IL_FALSE;
							//inertial_criticalSection_leave(&AHRSInt->critSecForResponseMatchAccess);


						}
						else {
							/* We received data but we have not found the starting point of a command. */

						}
					}
				}

				break;
			}
			case IL_AHRS_GET_BIT_CMD:

			case IL_AHRS_GET_VER_FIRMWARE_CMD:

			case IL_AHRS_GETDATA_REQ_CMD:

			case IL_READ_AHRS_PAR_RECEIVE:
			{
				AHRS_processReceivedPacket(ahrs, readBuffer, AHRSInt->num_bytes_recive);
			}
			case IL_AHRS_NMEA_CONT_CMD:
			case IL_AHRS_NMEA_REQ_CMD:
			{
			
				for (int i = 0; i < numOfBytesRead; i++)
				{
						printf("%c", readBuffer[i]);
				}
				
			}
			default:
#if IL_AHRS_DBG
				printf("No CMD flag Set !!!! \n");
#endif
				break;
			}

		}
	}
	inertial_event_signal(AHRSInt->waitForThreadToStopServicingComPortEvent);

	return IL_NULL;

}


IL_BOOL AHRS_verifyConnectivity(IL_AHRS* ahrs)
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


IL_ERROR_CODE AHRS_ReadInternalParameters(IL_AHRS* ahrs, AHRS_SetInternalData* data)
{

	int i = 6;
	AHRSInternal* AHRSInt;
	AHRSInt = AHRS_10_getInternalData(ahrs);

	if (AHRSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	//printf("0x%04x", *(uint16_t*)(&(AHRSInt->dataBuffer[i+0])));

	data->Data_rate = (double)(*(uint16_t*)(&(AHRSInt->dataBuffer[i + 0])));

	data->Initial_Alignment_Time = (double)(*(uint16_t*)(&(AHRSInt->dataBuffer[i + 2])));

	data->Magnetic_Declination = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 4]))) ;

	data->Latitude = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 8]))) ;

	data->Longitude = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 12]))) ;

	data->Altitude = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 16]))) ;

	data->Date = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 20])));

	// data->Year = (int)(*(int8_t*)(&(AHRSInt->dataBuffer[i + 20])));

	// data->Month = (int)(*(int8_t*)(&(AHRSInt->dataBuffer[i + 21])));

	// data->Day = (int)(*(int8_t*)(&(AHRSInt->dataBuffer[i + 22])));

	data->Alignment_Angle_A1 = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 24]))) ;

	data->Alignment_Angle_A2 = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 28]))) ;

	data->Alignment_Angle_A3 = (double)(*(float_t*)(&(AHRSInt->dataBuffer[i + 32]))) ;

	//data->Device_Id = (char *)(*(int64_t*)(&(AHRSInt->dataBuffer[i + 36])));

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


AHRSInternal* AHRS_getInternalData(IL_AHRS* ahrs)
{
	return (AHRSInternal*)ahrs->internalData;
}

IL_ERROR_CODE AHRS_transaction(IL_AHRS* ahrs, unsigned char *cmdToSend, int responseMatch)
{
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);


	AHRS_enableResponseChecking_threadSafe(ahrs, responseMatch);

	AHRS_writeData_threadSafe(ahrs, cmdToSend, sizeof(cmdToSend));

	return inertial_event_waitFor(AHRSInt->waitForCommandResponseEvent, AHRSInt->timeout);
}

void AHRS_enableResponseChecking_threadSafe(IL_AHRS* ahrs, int responseMatch)
{
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRSInt->critSecForResponseMatchAccess);

	AHRSInt->checkForResponse = IL_TRUE;
	AHRSInt->cmdResponseSize = responseMatch;

	inertial_criticalSection_leave(&AHRSInt->critSecForResponseMatchAccess);
}


void AHRS_processReceivedPacket(IL_AHRS* ahrs, unsigned char buffer[], int num_bytes_to_recive)
{
	AHRSInternal* AHRSInt;


	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRSInt->critSecForLatestAsyncDataAccess);
	AHRSInt->dataBuffer = buffer;
	AHRSInt->recive_flag = ILERR_DATA_IN_BUFFER;
	inertial_criticalSection_leave(&AHRSInt->critSecForLatestAsyncDataAccess);

#if  IL_RAW_DATA
	for (int i = 0; i < AHRSInt->num_bytes_recive; i++)
	{
		printf("0x%02x ", AHRSInt->dataBuffer[i]);
	}
	printf("\n");

#endif

#if 0
	/* See if we should be checking for a command response. */
	if (AHRS_shouldCheckForResponse_threadSafe(ahrs, buffer, num_bytes_to_recive)) {

		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		//if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			 //AHRS_disableResponseChecking_threadSafe(ahrs);

			 /* The line below should be thread-safe since the user thread should be
			  * blocked until we signal that we have received the response. */


			  /* Signal to the user thread we have received a response. */
		inertial_event_signal(AHRSInt->waitForCommandResponseEvent);
		//}
	}
#endif 

}


IL_ERROR_CODE AHRS_YPR(IL_AHRS* ahrs, AHRSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	AHRSInternal* AHRSInt;

	printf("inside AHRS_YPR\n");

	AHRSInt = AHRS_getInternalData(ahrs);

	if (AHRSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_CONT_1_CMD:
	case IL_AHRS_CONT_2_CMD:
	case IL_AHRS_CONT_3_CMD:
	case IL_AHRS_REQ_1_CMD:
	case IL_AHRS_REQ_2_CMD:
	case IL_AHRS_REQ_3_CMD:
	{

		i = 6;


		data->ypr.yaw = (double)(*(uint16_t*)(&(AHRSInt->dataBuffer[i + 0]))) / 100;
		data->ypr.pitch = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / 100;
		data->ypr.roll = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 4]))) / 100;

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


IL_ERROR_CODE AHRS_getGyroAccMag(IL_AHRS* ahrs, AHRSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int kg = 50;
	int ka = 4000;
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

	if (AHRSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_CONT_1_CMD:
	case IL_AHRS_REQ_1_CMD:
	case IL_AHRS_CONT_3_CMD:
	case IL_AHRS_REQ_3_CMD:
	{

	
		i =  12 ;

		data->gyro.c0 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 0]))) / kg;
		data->gyro.c1 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / kg;
		data->gyro.c2 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 4]))) / kg;


		i = i + 6;

		data->acceleration.c0 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 0]))) / ka;
		data->acceleration.c1 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / ka;
		data->acceleration.c2 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 4]))) / ka;

		i = i + 6;

		data->magnetic.c0 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 4]))) * 10;

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

IL_ERROR_CODE AHRS_getQuaternionData(IL_AHRS* ahrs, AHRSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int quat_conv = 10000;
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_10_getInternalData(ahrs);

	if (AHRSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;


	switch (ahrs->cmd_flag)
	{

	case IL_AHRS_CONT_2_CMD:
	case IL_AHRS_REQ_2_CMD:
	{
		i = 12;

		data->quaternion.Lk0 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 0]))) / quat_conv;
		data->quaternion.Lk1 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / quat_conv;
		data->quaternion.Lk2 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 4]))) / quat_conv;
		data->quaternion.Lk3 = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 6]))) / quat_conv;

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

IL_ERROR_CODE AHRS_getSensorData(IL_AHRS* ahrs, AHRSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

	if (AHRSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_CONT_1_CMD:
	case IL_AHRS_CONT_2_CMD:
	case IL_AHRS_CONT_3_CMD:
	case IL_AHRS_REQ_1_CMD:
	case IL_AHRS_REQ_2_CMD:
	case IL_AHRS_REQ_3_CMD:
	{

		//i = (ahrs->cmd_flag == IL_AHRS_GA_DATA_RECEIVE) ? 34 : (ahrs->cmd_flag == IL_AHRS_ORIENTATION_RECEIVE) ? 36 : 32;
        i = 36;
		data->Vinp = (double)(*(uint16_t*)(&(AHRSInt->dataBuffer[i + 0]))) / 100;
		data->Temper = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / 10;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case :
	{
		i = 24;
		data->Temper = (double)(*(int16_t*)(&(AHRSInt->dataBuffer[i + 2]))) / 10;
		data->Vinp = 0;
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

IL_ERROR_CODE AHRS_Stop(IL_AHRS* ahrs)
{
	int errorCode;
	printf("Stopping the AHRS device \n");
	unsigned char stop_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xFE ,0x05, 0x01 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("size of stop %d \n" , sizeof(stop_payload));
	ahrs->cmd_flag = IL_AHRS_STOP_CMD;
	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, stop_payload, sizeof(stop_payload));
	return errorCode;
}

IL_ERROR_CODE AHRScont1_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRScont1_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x80 ,0x87 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRScont1_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRScont1_payload, sizeof(AHRScont1_payload));

	return errorCode;
}

IL_ERROR_CODE AHRScont2_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRScont2_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x82 ,0x89 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRScont2_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRScont2_payload, sizeof(AHRScont2_payload));

	return errorCode;
}

IL_ERROR_CODE AHRScont3_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRScont3_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x83 ,0x8A ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRScont3_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRScont3_payload, sizeof(AHRScont3_payload));

	return errorCode;
}

IL_ERROR_CODE AHRSreq1_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRSreq1_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x84 ,0x8B ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRSreq1_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRSreq1_payload, sizeof(AHRSreq1_payload));

	return errorCode;
}

IL_ERROR_CODE AHRSreq2_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRSreq2_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x86 ,0x8D ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRSreq1_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRSreq2_payload, sizeof(AHRSreq2_payload));

	return errorCode;
}

IL_ERROR_CODE AHRSreq3_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  AHRSreq3_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x87 ,0x8E ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRSreq3_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, AHRSreq3_payload, sizeof(AHRSreq3_payload));

	return errorCode;
}

IL_ERROR_CODE NMEAcont_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  NMEAcont_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x88 ,0x8F ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside NMEAcont_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, NMEAcont_payload, sizeof(NMEAcont_payload));

	return errorCode;
}

IL_ERROR_CODE NMEAreq_Receive(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  NMEAreq_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x89 ,0x90 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside NMEAreq_Receive \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, NMEAreq_payload, sizeof(NMEAreq_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_LowPowerOn(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  LowPowerOn_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xB0 ,0xB7 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_LowPowerOn \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, LowPowerOn_payload, sizeof(LowPowerOn_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_LowPowerOff(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  LowPowerOff_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xBA ,0xC1 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_LowPowerOff \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, LowPowerOff_payload, sizeof(LowPowerOff_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_GetVerFirmware(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  GetVerFirmware_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x1F ,0x26 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_GetVerFirmware \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, GetVerFirmware_payload, sizeof(GetVerFirmware_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_GetBIT(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  GetBIT_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x1A ,0x21 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_GetBIT \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, GetBIT_payload, sizeof(GetBIT_payload));

	return errorCode;
}

IL_ERROR_CODE ReadAHRSpar(IL_AHRS* ahrs)
{
	int errorCode;
	unsigned char  ReadAHRSpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x41 ,0x48 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside ReadAHRSpar \n");

	ahrs->cmd_flag = IL_READ_AHRS_PAR_RECEIVE;

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, ReadAHRSpar_payload, sizeof(ReadAHRSpar_payload));


	return errorCode;
}

IL_ERROR_CODE LoadAHRSpar(IL_AHRS* ahrs)
{
	int errorCode;

	unsigned char  LoadAHRSpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x40 ,0x47 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	printf("inside LoadAHRSpar \n");

	ahrs->cmd_flag = IL_LOAD_AHRS_PAR_RECEIVE;

	//AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, LoadAHRSpar_payload, sizeof(LoadAHRSpar_payload));

	return errorCode;
}

IL_ERROR_CODE AHRS_GetDataReq(IL_AHRS* ahrs)
{
	int errorCode;

	unsigned char  GetDataReq_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xCA ,0xD1 ,0x00 };

	if (!ahrs->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("inside AHRS_GetDataReq \n");

	AHRS_Recive_size(ahrs);
	errorCode = AHRS_writeOutCommand(ahrs, GetDataReq_payload, sizeof(GetDataReq_payload));

	return errorCode;
}


void AHRS_Recive_size(IL_AHRS* ahrs)
{
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);
#if IL_AHRS_DBG
	printf("now ahrs->cmd_flag : %d", ahrs->cmd_flag);
#endif
	switch (ahrs->cmd_flag)
	{
	case IL_AHRS_STOP_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_STOP_CMD_RECEIVE_SIZE;
		break;
	case IL_AHRS_CONT_1_CMD:
	case IL_AHRS_REQ_1_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_FULL_OUTPUT_RECEIVE_SIZE;
		break;
	case IL_AHRS_CONT_2_CMD:
	case IL_AHRS_REQ_2_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_ORIENTATION_OUTPUT_RECEIVE_SIZE;
		break;
	case IL_AHRS_CONT_3_CMD:
	case IL_AHRS_REQ_3_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_ORIENTATION_SENSOR_OUTPUT_RECEIVE_SIZE;
		break;
	case IL_AHRS_GET_BIT_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_GET_BIT_RECEIVE_SIZE;
		break;
	case IL_AHRS_GET_VER_FIRMWARE_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_GET_FIRMWARE_VER_RECEIVE_SIZE;
		break;
	case IL_AHRS_NMEA_CONT_CMD:
	case IL_AHRS_NMEA_REQ_CMD:
		AHRSInt->num_bytes_recive = IL_AHRS_NMEA_CMD_RECEIVE_SIZE;
		break;
	case IL_READ_AHRS_PAR_RECEIVE:
		AHRSInt->num_bytes_recive = IL_READ_AHRS_PAR_CMD_RECEIVE_SIZE;
		break;
	case IL_LOAD_AHRS_PAR_RECEIVE:
		AHRSInt->num_bytes_recive = IL_LOAD_AHRS_PAR_CMD_RECEIVE_SIZE;
		break;
	default:
		break;
	}
}

IL_BOOL AHRS_shouldCheckForResponse_threadSafe(IL_AHRS* ahrs, char* responseMatchBuffer, int num_bytes_to_recive)
{
	AHRSInternal* AHRSInt;
	IL_BOOL shouldCheckResponse;

	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRSInt->critSecForResponseMatchAccess);
	shouldCheckResponse = 1; // to avoid warning now
	inertial_criticalSection_leave(&AHRSInt->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}


void AHRS_disableResponseChecking_threadSafe(IL_AHRS* ahrs)
{
	AHRSInternal* AHRSInt;

	AHRSInt = AHRS_getInternalData(ahrs);

	inertial_criticalSection_enter(&AHRSInt->critSecForResponseMatchAccess);

	AHRSInt->checkForResponse = IL_FALSE;
	AHRSInt->cmdResponseSize = 0;
	AHRSInt->cmdResponseMatchBuffer[0] = 0;

	inertial_criticalSection_leave(&AHRSInt->critSecForResponseMatchAccess);
}


IL_ERROR_CODE AHRS_registerDataReceivedListener(IL_AHRS* ahrs, AHRSNewDataReceivedListener listener)
{
	AHRSInternal* AHRSInt = AHRS_getInternalData(ahrs);

	if (AHRSInt->DataListener != NULL)
		return ILERR_UNKNOWN_ERROR;

	AHRSInt->DataListener = listener;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE AHRS_unregisterDataReceivedListener(IL_AHRS* ahrs, AHRSNewDataReceivedListener listener)
{
	AHRSInternal* AHRSInt = AHRS_getInternalData(ahrs);

	if (AHRSInt->DataListener == NULL)
		return ILERR_UNKNOWN_ERROR;

	if (AHRSInt->DataListener != listener)
		return ILERR_UNKNOWN_ERROR;

	AHRSInt->DataListener = NULL;

	return ILERR_NO_ERROR;
}

unsigned char AHRS_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char)cmdToCheck[i];

	return xorVal;
}

void AHRS_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = AHRS_checksum_compute(cmdToCheck);
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

void AHRS_processAsyncData(IL_AHRS* ahrs, unsigned char buffer[])
{
	AHRSCompositeData data;
	AHRSInternal* AHRSInt = AHRS_getInternalData(ahrs);

	memset(&data, 0, sizeof(AHRSCompositeData));


	/* We had an async data packet and need to move it to AHRSInt->lastestAsyncData. */
	inertial_criticalSection_enter(&AHRSInt->critSecForLatestAsyncDataAccess);
	memcpy(&AHRSInt->lastestAsyncData, &data, sizeof(AHRSCompositeData));
	inertial_criticalSection_leave(&AHRSInt->critSecForLatestAsyncDataAccess);

	if (AHRSInt->DataListener != NULL)
		AHRSInt->DataListener(ahrs, &AHRSInt->lastestAsyncData);


}




/** \endcond */