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
#include <math.h>
#include <stdint.h>
#include "InertialLabs_INS.h"
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
	 * be checking to a command response comming from the INS device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the INS. This field should only be accessed in the
	 * functions INS_shouldCheckForResponse_threadSafe,
	 * INS_enableResponseChecking_threadSafe, and
	 * INS_disableResponseChecking_threadSafe.
	 */
	IL_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the INS is a match for the command sent.
	 * This field should only be accessed in the functions
	 * INS_shouldCheckForResponse_threadSafe, INS_enableResponseChecking_threadSafe,
	 * and INS_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[IL_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the INS device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "INS_SensorsData" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[IL_MAX_RESPONSE_SIZE + 1];

	unsigned char* dataBuffer;
	//unsigned char           dataBuffer[READ_BUFFER_SIZE];
	int 			        cmdResponseSize;

	int 					num_bytes_recive;
	int 					recive_flag;

	int						initial_align_running;

	INSCompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	INSNewDataReceivedListener DataListener;

} INSInternal;

/* Private type definitions. *************************************************/


void* INS_communicationHandler(void*);

IL_ERROR_CODE  INS_writeOutCommand(IL_INS* ins, unsigned char cmdToSend[], int cmdSize);


/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	ins	Pointer to the IL_INS control object.
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
IL_BOOL INS_shouldCheckForResponse_threadSafe(IL_INS* ins, char* responseMatchBuffer, int num_bytes_to_recive);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	ins	Pointer to the IL_INS control object.
 *
 * \param[in]	responseMatch
 * \\ TODO: add the comments here
 */
void INS_enableResponseChecking_threadSafe(IL_INS* ins, int responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	ins 	Pointer to the IL_INS control object.
 */
void INS_disableResponseChecking_threadSafe(IL_INS* ins);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command  and transmits it to the device.
 * The function will then wait until the response is received. The response
 * will be located in the INSInternal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	ins				Pointer to the IL_INS control object.
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
IL_ERROR_CODE INS_transaction(IL_INS* ins, unsigned char cmdToSend[], int responseMatch);


/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions  inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int INS_writeData_threadSafe(IL_INS* ins, unsigned char dataToSend[], unsigned int dataLength);


/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
IL_ERROR_CODE INS_readData_threadSafe(IL_INS* ins, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a INS control object.
 *
 * \param[in]	INS	Pointer to the INS control object.
 * \return The internal data.
 */
INSInternal* INS_getInternalData(IL_INS* ins);

void INS_processAsyncData(IL_INS* ins, unsigned char buffer[]);

/**
 * \brief  Process and Check the received packaet and transfer in to the internal structure.
 *
 * \param[in]	INS	Pointer to the INS control object.
 * \param[in]   data buffer
 * \param[in]   Number of bytes recieved from the INS
 *
 * \return Copy the data buffer to internal data structure variable.
 */
void INS_processReceivedPacket(IL_INS* ins, unsigned char buffer[], int num_bytes_to_recive);

/**
 * \brief Helper method to set the buffer size in the INS control object,according to the command.
 *
 * \param[in]	INS	Pointer to the INS control object.
 * \return buffer size in  internal data structure.
 */
void INS_Recive_size(IL_INS* ins);


/* Function definitions. *****************************************************/


IL_ERROR_CODE INS_connect(IL_INS* newINS, const char* portName, int baudrate)
{

	INSInternal* INSInt;
	IL_ERROR_CODE errorCode;

	/* Allocate memory. */
	INSInt = (INSInternal*)malloc(sizeof(INSInternal));
	newINS->internalData = INSInt;

	newINS->portName = (char*)malloc(strlen(portName) + 1);
	strcpy(newINS->portName, portName);
	newINS->baudRate = baudrate;
	newINS->isConnected = IL_FALSE;
	INSInt->continueServicingComPort = IL_TRUE;
	INSInt->checkForResponse = IL_FALSE;
	INSInt->timeout = DEFAULT_TIMEOUT_IN_MS;
	INSInt->DataListener = NULL;

	memset(&INSInt->lastestAsyncData, 0, sizeof(INSCompositeData));


	errorCode = inertial_comPort_open(&INSInt->comPortHandle, portName, baudrate);

	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_criticalSection_initialize(&INSInt->criticalSection);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&INSInt->critSecForComPort);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&INSInt->critSecForResponseMatchAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&INSInt->critSecForLatestAsyncDataAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_event_create(&INSInt->waitForThreadToStopServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&INSInt->waitForCommandResponseEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&INSInt->waitForThreadToStartServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_thread_startNew(&INSInt->comPortServiceThreadHandle, &INS_communicationHandler, newINS);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	newINS->isConnected = IL_TRUE;

	errorCode = inertial_event_waitFor(INSInt->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;


	return ILERR_NO_ERROR;
}


IL_ERROR_CODE INS_disconnect(IL_INS* ins)
{
	INSInternal* INSInt;

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;

	INSInt = INS_getInternalData(ins);

	INSInt->continueServicingComPort = IL_FALSE;

	inertial_event_waitFor(INSInt->waitForThreadToStopServicingComPortEvent, -1);
	INS_Stop(ins);
	//sleep(3);

	inertial_comPort_close(INSInt->comPortHandle);

	inertial_criticalSection_dispose(&INSInt->criticalSection);
	inertial_criticalSection_dispose(&INSInt->critSecForComPort);
	inertial_criticalSection_dispose(&INSInt->critSecForResponseMatchAccess);
	inertial_criticalSection_dispose(&INSInt->critSecForLatestAsyncDataAccess);


	/* Free the memory associated with the INS structure. */
	//free(&INSInt->waitForThreadToStopServicingComPortEvent);
	//free(&INSInt->waitForCommandResponseEvent);
	//free(&INSInt->waitForThreadToStartServicingComPortEvent);
	free(ins->internalData);

	ins->isConnected = IL_FALSE;
#if defined(WIN32) || defined(WIN64) 
	Sleep(3000);
#else
	sleep(3);
#endif 

	return ILERR_NO_ERROR;
}


int INS_writeData_threadSafe(IL_INS* ins, unsigned char dataToSend[], unsigned int dataLength)
{
	int errorCode;
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

#if IL_DBG
	printf("INS_writeData_threadSafe : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n\n\t", dataToSend[0], dataToSend[1], dataToSend[2],
		dataToSend[3], dataToSend[4], dataToSend[5],
		dataToSend[6], dataToSend[7], dataToSend[8]);

#endif 
	inertial_criticalSection_enter(&INSInt->critSecForComPort);
	errorCode = inertial_comPort_writeData(INSInt->comPortHandle, dataToSend, dataLength);
	inertial_criticalSection_leave(&INSInt->critSecForComPort);

	//printf("after read");

	return errorCode;
}

IL_ERROR_CODE INS_readData_threadSafe(IL_INS* ins, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	IL_ERROR_CODE errorCode;
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

	inertial_criticalSection_enter(&INSInt->critSecForComPort);
	errorCode = inertial_comPort_readData(INSInt->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	inertial_criticalSection_leave(&INSInt->critSecForComPort);

	if (ins->mode)
	{

		if ((ins->cmd_flag == IL_OPVT_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_QPVT_RECEIVE && *numOfBytesActuallyRead == (int)IL_QPVT_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVT2A_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT2A_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVT2AW_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT2AW_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVT2AHR_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT2AHR_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVTAD_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVTAD_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_MINIMAL_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_MINIMAL_DATA_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_SENSOR_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_SENSOR_DATA_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVT_RAWIMU_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT_RAWIMU_DATA_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_OPVT_GNSSEXT_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_OPVT_GNSSEXT_DATA_CMD_RECEIVE_SIZE) ||
			(ins->cmd_flag == IL_SPAN_RAWIMU_RECEIVE && *numOfBytesActuallyRead == (int)IL_SPAN_RAWIMU_CMD_RECEIVE_SIZE))
		{
			return ILERR_NO_ERROR;
		}
		else if (ins->cmd_flag == IL_STOP_CMD && *numOfBytesActuallyRead == IL_STOP_CMD_RECEIVE_SIZE)
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

IL_ERROR_CODE INS_writeOutCommand(IL_INS* ins, unsigned char cmdToSend[], int cmdSize)
{
	//char packetTail[] = {0xFF ,0xFF};
	 //printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  %d \n", cmdToSend[0] ,cmdToSend[1] ,cmdToSend[2],
	 //																				cmdToSend[3] ,cmdToSend[4],cmdToSend[5],
	 //																				cmdToSend[6] ,cmdToSend[7],cmdToSend[8], cmdSize);

	 //printf("size of : %d \n ", strlen(cmdToSend));
	INS_writeData_threadSafe(ins, cmdToSend, cmdSize);

	//
	//INS_writeData_threadSafe(ins,packetTail, strlen(packetTail));

	return ILERR_NO_ERROR;
}


void* INS_communicationHandler(void* INSobj)
{

	IL_INS* ins;
	INSInternal* INSInt;
	unsigned char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	unsigned char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	IL_BOOL haveFoundStartOfCommand = IL_FALSE;

	memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);

	ins = (IL_INS*)INSobj;
	INSInt = INS_getInternalData(ins);

	inertial_event_signal(INSInt->waitForThreadToStartServicingComPortEvent);

	while (INSInt->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		INS_readData_threadSafe(ins, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);

		if (numOfBytesRead == 0)
		{
			// There was no data. Sleep for a short amount of time before continuing.
			inertial_sleepInMs(NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_NOT_RECEIVING_ON_COM_PORT);
			continue;
		}
		else
		{
#if IL_DBG
			for (int i = 0; i < numOfBytesRead; i++)
			{
				printf("0x%02x  ", readBuffer[i]);
			}
			printf("\n");
			printf("numOfBytesRead %d : \n", numOfBytesRead);
			printf("number of bytes to recive : %d \n", INSInt->num_bytes_recive);
			printf(" cmd_flag : %d \n", ins->cmd_flag);
#endif 

			switch (ins->cmd_flag)
			{
			case IL_STOP_CMD:
#if IL_DBG
				printf("IL_STOP_CMD \n");
# endif
				break;

			case IL_SET_ONREQUEST_CMD:
#if IL_DBG
				printf("IL_SET_ONREQUEST_CMD \n");
#endif
				break;

			case IL_OPVT_RECEIVE:
			case IL_QPVT_RECEIVE:
			case IL_OPVT2A_RECEIVE:
			case IL_OPVT2AW_RECEIVE:
			case IL_OPVT2AHR_RECEIVE:
			case IL_OPVTAD_RECEIVE:
			case IL_MINIMAL_DATA_RECEIVE:
			case IL_OPVT_RAWIMU_DATA_RECEIVE:
			case IL_SENSOR_DATA_RECEIVE:
			case IL_OPVT_GNSSEXT_DATA_RECEIVE:
			case IL_SPAN_RAWIMU_RECEIVE:
			{
				//printf("cmd_flag value : %d \n" , ins->cmd_flag);
				if (ins->mode)
				{

					//inertial_criticalSection_enter(&INSInt->critSecForResponseMatchAccess);
					INS_processReceivedPacket(ins, readBuffer, INSInt->num_bytes_recive);
					//inertial_criticalSection_leave(&INSInt->critSecForResponseMatchAccess);
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

#if IL_DBG
							printf(" found the start byte \n");
							for (int i = 0; i < numOfBytesRead; i++)
							{
								printf("0x%02x  ", readBuffer[i]);
							}
							printf("\n");
#endif
							/* Alright, we have found the start of a command. */
							//free(INSInt->dataBuffer);
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

							if ((int)(readBuffer[curResponsePos + 4]) == 134 ||
								(int)(readBuffer[curResponsePos + 4]) == 56)
							{
								printf(" [INFO] Initial Alignment Done !!! \n");
								memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);
							}

						}
						if (((int)readBuffer[curResponsePos] == 170) && ((int)readBuffer[curResponsePos + 1] == 68) && ((int)readBuffer[curResponsePos + 2] == 18)) {

#if IL_DBG
							printf(" found the start byte \n");
							for (int i = 0; i < numOfBytesRead; i++)
							{
								printf("0x%02x  ", readBuffer[i]);
							}
							printf("\n");
#endif
							/* Alright, we have found the start of a command. */
							//free(INSInt->dataBuffer);
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

						}

						if (haveFoundStartOfCommand) {

							responseBuilderBuffer[responseBuilderBufferPos] = readBuffer[curResponsePos];
							responseBuilderBufferPos++;
							if (responseBuilderBufferPos == INSInt->num_bytes_recive)
							{
								haveFoundStartOfCommand = IL_FALSE;
							}
						}

						if (responseBuilderBufferPos == INSInt->num_bytes_recive)
						{

#if IL_DBG							
							printf(" one packet recive done ins continues mode \n");
#endif
							//inertial_criticalSection_enter(&INSInt->critSecForResponseMatchAccess);
							INS_processReceivedPacket(ins, responseBuilderBuffer, INSInt->num_bytes_recive);

							responseBuilderBufferPos = 0;
							haveFoundStartOfCommand = IL_FALSE;
							//inertial_criticalSection_leave(&INSInt->critSecForResponseMatchAccess);


						}
						else {
							/* We received data but we have not found the starting point of a command. */

						}
					}

				}
				break;
			}
			case IL_READ_INS_PAR_RECEIVE:
			{
				INS_processReceivedPacket(ins, readBuffer, INSInt->num_bytes_recive);
			}
			case IL_NMEA_RECEIVE:
			case IL_NMEA_SENSORS_RECEIVE:
			{
				if (ins->mode)
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
#if IL_DBG
				printf("no cmd flag set \n");
#endif 
				break;
			}
		}

	}
	inertial_event_signal(INSInt->waitForThreadToStopServicingComPortEvent);

	return IL_NULL;

}


IL_BOOL INS_verifyConnectivity(IL_INS* ins)
{
	//TODO 
	return true;
}
void textFromHexString(char* hex, char* result)
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

IL_ERROR_CODE INS_ReadRawImuParameters(IL_INS* ins, INSRawImuData* data)
{
	int i = 7;

	//pow(2,31)
	double conv_acc = pow(2, 31) / 200;
	double conv_gyro = pow(2, 31) / 720;

	INSInternal* INSInt;
	INSInt = INS_getInternalData(ins);

	data->port_address = (int)(*(uint8_t*)(&(INSInt->dataBuffer[i + 0])));

	data->message_length = (int)(*(uint16_t*)(&(INSInt->dataBuffer[i + 1])));

	data->sequence = (int)(*(uint16_t*)(&(INSInt->dataBuffer[i + 3])));

	data->idle_time = (int)(*(uint8_t*)(&(INSInt->dataBuffer[i + 5])));

	data->time_status = (int)(*(uint8_t*)(&(INSInt->dataBuffer[i + 6])));

	data->week = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 7])));

	data->gps_time = (double)(*(uint32_t*)(&(INSInt->dataBuffer[i + 9])));

	data->receiver_status = (double)(*(uint32_t*)(&(INSInt->dataBuffer[i + 13])));

	data->receiver_sw_version = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 19])));

	data->week_4byte = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 21])));

	data->gps_imu_time = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 25])));

	data->imu_status = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 33])));

	data->acceleration.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 37]))) / conv_acc;

	data->acceleration.c1 = -((double)(*(int32_t*)(&(INSInt->dataBuffer[i + 41]))) / conv_acc);

	data->acceleration.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 45]))) / conv_acc;

	data->gyro.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 49]))) / conv_gyro;

	data->gyro.c1 = -((double)(*(int32_t*)(&(INSInt->dataBuffer[i + 53]))) / conv_gyro);

	data->gyro.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 57]))) / conv_gyro;

	printf("\nPort Address:  %d\n", data->port_address);

	printf("\nMessage Length: %d\n", data->message_length);

	printf("\n Sequence :  %d\n", data->sequence);

	printf("\nTime Status: %d\n", data->time_status);


	printf("\nWeek: %f\n", data->week);

	printf("\nGPS Time: %f\n", data->gps_time);

	printf("\nReceiver Status: %f\n", data->receiver_status);

	printf("\nReceiver S/W Version: %f\n", data->receiver_sw_version);

	printf("\nWeek (4 byte unsigned integer): %f\n", data->week_4byte);


	printf(
		"\n Acceleration : \n"
		"   AccX  :%f\n"
		"   AccY  :%f\n"
		"  	AccZ  :%f\n",
		data->acceleration.c0,
		data->acceleration.c1,
		data->acceleration.c2);

	printf(
		"\n Gyro : \n"
		"   GyroX  :%f\n"
		"   GyroY  :%f\n"
		"  	GyroZ  :%f\n",
		data->gyro.c0,
		data->gyro.c1,
		data->gyro.c2);

#ifdef IL_DECODE_DATA

	//for GPS Reference Time Status
	switch (data->time_status)
	{
	case 20:
		printf("Time validity is unknown");
		break;
	case 60:
		printf("Time is set approximately");
		break;
	case 80:
		printf("Time is approaching coarse precision");
		break;
	case 100:
		printf("Time is valid to coarse precision");
		break;
	case 120:
		printf("Time is coarse set and is being steered");
		break;
	case 130:
		printf("Position is lost and the range bias cannot be calculated");
		break;
	case 140:
		printf("Time is adjusting to fine precision");
		break;
	case 160:
		printf("Time has fine precision");
		break;
	case 170:
		printf("Time is fine set and is being steered by the backup system");
		break;
	case 180:
		printf("Time is fine set and is being steered");
		break;

	default:
		printf("check your data once its not in the range!!!! : %d", data->time_status);
		break;
	}


#endif


	return ILERR_NO_ERROR;


}

IL_ERROR_CODE INS_Alignment(IL_INS* ins, INSSetInitialData* data)
{
	INSInternal* INSInt;
	INSInt = INS_getInternalData(ins);

	int i = 4;

	int data_size = (int)(*(int16_t*)(&(INSInt->dataBuffer[i + 0])));

	data->Gyros_bias1 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 2])));

	data->Gyros_bias2 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 6])));

	data->Gyros_bias3 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 10])));

	data->Average_acceleration1 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 14])));

	data->Average_acceleration2 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 18])));

	data->Average_acceleration3 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 22])));

	data->Average_magnfield1 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 26])));

	data->Average_magnfield2 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 30])));

	data->Average_magnfield3 = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 34])));

	data->Initial_Heading = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 38])));

	data->Initial_Roll = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 42])));

	data->Initial_Pitch = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 46])));


	if (data_size == 128)
	{
		data->temp_pressure_sensor = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 52])));

		data->pressure_data = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 56])));

		data->temp_gyro1 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 60])));

		data->temp_gyro2 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 62])));

		data->temp_gyro3 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 64])));

		data->temp_acc1 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 66])));

		data->temp_acc2 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 68])));

		data->temp_acc3 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 70])));

		data->temp_mag1 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 72])));

		data->temp_mag2 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 74])));

		data->temp_mag3 = (float)(*(int16_t*)(&(INSInt->dataBuffer[i + 76])));

		data->Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 78])));

		data->Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 86])));

		data->Altitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 94])));

		data->Velocity_V_east = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 102])));

		data->Velocity_V_north = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 106])));

		data->Velocity_V_up = (float)(*(int32_t*)(&(INSInt->dataBuffer[i + 110])));

		data->Velocity_V_up = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 114])));


	}


	return ILERR_NO_ERROR;
}

IL_ERROR_CODE INS_ReadInternalParameters(IL_INS* ins, INSSetInternalData* data)
{

	int i = 6;

	int m_conv = 100;
	int deg_conv = 10000000;

	INSInternal* INSInt;
	INSInt = INS_getInternalData(ins);

	//printf("0x%04x", *(uint16_t*)(&(INSInt->dataBuffer[i+0])));

	data->Data_rate = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0])));

	data->Initial_Alignment_Time = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 2])));

	data->Magnetic_Declination = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / m_conv;

	data->Latitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / deg_conv;

	data->Longitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 12]))) / deg_conv;

	data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 16]))) / m_conv;

	data->Year = (int)(*(int8_t*)(&(INSInt->dataBuffer[i + 20])));

	data->Month = (int)(*(int8_t*)(&(INSInt->dataBuffer[i + 21])));

	data->Day = (int)(*(int8_t*)(&(INSInt->dataBuffer[i + 22])));

	data->Alignment_Angle_A1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 23]))) / m_conv;

	data->Alignment_Angle_A2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 25]))) / m_conv;

	data->Alignment_Angle_A3 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 27]))) / m_conv;

	data->INS_Mount_Right = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 29]))) / m_conv;

	data->INS_Mount_Forward = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 31]))) / m_conv;

	data->INS_Mount_Up = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 33]))) / m_conv;

	data->Antenna_Pos_Right = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 35]))) / m_conv;

	data->Antenna_Pos_Forward = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 37]))) / m_conv;

	data->Antenna_Pos_Up = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 39]))) / m_conv;


	data->Altitude_one_byte = (int)(*(int8_t*)(&(INSInt->dataBuffer[i + 41])));


	data->Baro_Altimeter = (int)(*(int8_t*)(&(INSInt->dataBuffer[i + 58])));


	//ROS_INFO("\nINS Device Name: %s\n",data.INS_Device_Name);

	printf("\nDate:  %d - %d - %d\n", data->Year, data->Month, data->Day);

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
		"\n INS_Mount_Right : %f\n"
		"  INS_Mount_Forward : %f\n"
		"  INS_Mount_Up  : %f\n",
		data->INS_Mount_Right,
		data->INS_Mount_Forward,
		data->INS_Mount_Up);

	printf(
		"\n Alignment_Angle_A1 : %f\n"
		"  Alignment_Angle_A2 : %f\n"
		"  Alignment_Angle_A3  : %f\n",
		data->Alignment_Angle_A1,
		data->Alignment_Angle_A2,
		data->Alignment_Angle_A3);

	printf(
		"\n Antenna_Pos_Right : %f\n"
		"  Antenna_Pos_Forward : %f\n"
		"  Antenna_Pos_Up  : %f\n",
		data->Antenna_Pos_Right,
		data->Antenna_Pos_Forward,
		data->Antenna_Pos_Up);

	printf("\n Baro_altimeter : %d \n ", data->Baro_Altimeter);

	//INSInt->dataBuffer = NULL;

	return ILERR_NO_ERROR;

}


INSInternal* INS_getInternalData(IL_INS* ins)
{
	return (INSInternal*)ins->internalData;
}

IL_ERROR_CODE INS_transaction(IL_INS* ins, unsigned char* cmdToSend, int responseMatch)
{
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);


	INS_enableResponseChecking_threadSafe(ins, responseMatch);

	INS_writeData_threadSafe(ins, cmdToSend, sizeof(cmdToSend));

	return inertial_event_waitFor(INSInt->waitForCommandResponseEvent, INSInt->timeout);
}

void INS_enableResponseChecking_threadSafe(IL_INS* ins, int responseMatch)
{
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

	inertial_criticalSection_enter(&INSInt->critSecForResponseMatchAccess);

	INSInt->checkForResponse = IL_TRUE;
	INSInt->cmdResponseSize = responseMatch;

	inertial_criticalSection_leave(&INSInt->critSecForResponseMatchAccess);
}


void INS_processReceivedPacket(IL_INS* ins, unsigned char buffer[], int num_bytes_to_recive)
{
	INSInternal* INSInt;

	//INSInt->dataBuffer = NULL;

	INSInt = INS_getInternalData(ins);

	inertial_criticalSection_enter(&INSInt->critSecForLatestAsyncDataAccess);
	INSInt->dataBuffer = buffer;
	INSInt->recive_flag = ILERR_DATA_IN_BUFFER;
	inertial_criticalSection_leave(&INSInt->critSecForLatestAsyncDataAccess);

#if  IL_RAW_DATA
	for (int i = 0; i < INSInt->num_bytes_recive; i++)
	{
		printf("0x%02x ", INSInt->dataBuffer[i]);
	}
	printf("\n");

#endif

#if 0
	/* See if we should be checking for a command response. */
	if (INS_shouldCheckForResponse_threadSafe(ins, buffer, num_bytes_to_recive)) {

		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		//if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			 //INS_disableResponseChecking_threadSafe(ins);

			 /* The line below should be thread-safe since the user thread should be
			  * blocked until we signal that we have received the response. */


			  /* Signal to the user thread we have received a response. */
		inertial_event_signal(INSInt->waitForCommandResponseEvent);
		//}
	}
#endif 

}

IL_ERROR_CODE INS_YPR(IL_INS* ins, INSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	INSInternal* INSInt;

	//printf("inside INS_YPR\n");

	INSInt = INS_getInternalData(ins);
	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT_RECEIVE:
	case IL_SENSOR_DATA_RECEIVE:
	case IL_OPVT2A_RECEIVE:
	case IL_OPVT2AW_RECEIVE:
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVTAD_RECEIVE:
	case IL_MINIMAL_DATA_RECEIVE:
	{
		i = 6;

		data->ypr.yaw = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / 100;
		data->ypr.pitch = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / 100;
		data->ypr.roll = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) / 100;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_RAWIMU_DATA_RECEIVE:
	{
		i = 48;

		data->ypr.yaw = (double)(*(uint32_t*)(&(INSInt->dataBuffer[i + 0]))) / 1000;
		data->ypr.pitch = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 1000;
		data->ypr.roll = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 1000;

		errorCode = ILERR_NO_ERROR;
		break;

	}
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
	{
		i = 10;
		data->ypr.yaw = (double)(*(uint32_t*)(&(INSInt->dataBuffer[i + 0]))) / 1000000;
		data->ypr.pitch = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 1000000;
		data->ypr.roll = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 1000000;

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

IL_ERROR_CODE INS_getGyroAccMag(IL_INS* ins, INSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int kg = 50;
	int ka = 4000;
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT_RECEIVE:
	case IL_QPVT_RECEIVE:
	case IL_OPVT2A_RECEIVE:
	case IL_OPVT2AW_RECEIVE:
	case IL_SENSOR_DATA_RECEIVE:
	{
		i = (ins->cmd_flag == IL_QPVT_RECEIVE) ? 14 : 12;

		data->gyro.c0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) / kg;
		data->gyro.c1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / kg;
		data->gyro.c2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) / kg;


		i = i + 6;

		data->acceleration.c0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) / ka;
		data->acceleration.c1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / ka;
		data->acceleration.c2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) / ka;

		i = i + 6;

		data->magnetic.c0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) * 10;


		if (ins->cmd_flag == IL_SENSOR_DATA_RECEIVE)
		{
			i = i + 12;
			data->Vinp = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0])));
			data->Temper = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2])));
		}
		else
		{
			i = i + 8;

			data->Vinp = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / 100;
			data->Temper = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / 10;
		}


		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVTAD_RECEIVE:
	{
		i = 12;


		data->gyro.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 100000;
		data->gyro.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 100000;
		data->gyro.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 100000;

		i = 24;

		data->acceleration.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 1000000;
		data->acceleration.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 1000000;
		data->acceleration.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 1000000;

		i = 36;

		data->magnetic.c0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) * 10;

		i = 44;


		data->Vinp = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / 100;
		data->Temper = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / 10;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
	{
		i = 54;


		data->gyro.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 1000000;
		data->gyro.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 1000000;
		data->gyro.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 1000000;

		i = 66;

		data->acceleration.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 1000000;
		data->acceleration.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 1000000;
		data->acceleration.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 1000000;

		i = 78;

		data->magnetic.c0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) * 10;

		errorCode = ILERR_NO_ERROR;
		break;

	}
	case IL_OPVT_RAWIMU_DATA_RECEIVE:
	{
		i = 22;


		data->gyro.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 10000;
		data->gyro.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 10000;
		data->gyro.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 10000;

		i = 34;

		data->acceleration.c0 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / 10000;
		data->acceleration.c1 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / 10000;
		data->acceleration.c2 = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / 10000;

		data->magnetic.c0 = 0;
		data->magnetic.c1 = 0;
		data->magnetic.c2 = 0;

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
	return errorCode;
}

IL_ERROR_CODE INS_getPositionData(IL_INS* ins, INSPositionData* data)
{
	IL_ERROR_CODE errorCode;
	int i = 0; int j = 0;
	int deg_conv = 10000000;
	int deg_conv_8byte = 1000000000;
	int m_conv = 100;
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT2A_RECEIVE:
	case IL_OPVT2AW_RECEIVE:
	case IL_QPVT_RECEIVE:
	case IL_OPVT_RECEIVE:
	{
		//i = 36 ; j = 60;
		i = (ins->cmd_flag == IL_QPVT_RECEIVE) ? 38 : 36;
		j = i + 24;

		data->Latitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / deg_conv;
		data->Longitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / deg_conv;
		//data->Altitude = (double)( *(int32_t*)(&(INSInt->dataBuffer[i+8])) ) / m_conv;
		data->East_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 12]))) / m_conv;
		data->North_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 16]))) / m_conv;
		data->Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 20]))) / m_conv;

		data->GNSS_Latitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 0]))) / deg_conv;
		data->GNSS_Longitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 4]))) / deg_conv;
		data->GNSS_Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 8]))) / m_conv;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 8]))) / m_conv;
		data->GNSS_Horizontal_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 12]))) / m_conv;
		data->GNSS_Trackover_Ground = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 16]))) / m_conv;
		data->GNSS_Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 18]))) / m_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVTAD_RECEIVE:
	{
		i = 48;
		j = i + 32;

		data->Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 0]))) / deg_conv_8byte;
		data->Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 8]))) / deg_conv_8byte;
		//data->Altitude = (double)( *(int32_t*)(&(INSInt->dataBuffer[i+16])) ) / 1000;
		data->East_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 20]))) / m_conv;
		data->North_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 24]))) / m_conv;
		data->Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 28]))) / m_conv;

		data->GNSS_Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[j + 0]))) / deg_conv_8byte;
		data->GNSS_Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[j + 8]))) / deg_conv_8byte;
		data->GNSS_Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 16]))) / 1000;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 16]))) / 1000;
		data->GNSS_Horizontal_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 20]))) / m_conv;
		data->GNSS_Trackover_Ground = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 24]))) / m_conv;
		data->GNSS_Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 26]))) / m_conv;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_SENSOR_DATA_RECEIVE:
	{
		j = 40;

		data->GNSS_Latitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 0]))) / deg_conv;
		data->GNSS_Longitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 4]))) / deg_conv;
		data->GNSS_Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 8]))) / m_conv;

		j = 52;
		data->Latitude = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 0]))) / 1000;
		data->Longitude = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 2]))) / 1000;
		data->Altitude = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 4]))) / 1000;

		j = 58;
		data->GNSS_Horizontal_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 0]))) / m_conv;
		data->GNSS_Trackover_Ground = (double)(*(uint16_t*)(&(INSInt->dataBuffer[j + 4]))) / m_conv;
		data->GNSS_Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 6]))) / m_conv;

		errorCode = ILERR_NO_ERROR;
		break;

	}
	case IL_MINIMAL_DATA_RECEIVE:
	{
		i = 18;
		data->Latitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 0]))) / deg_conv;
		data->Longitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 4]))) / deg_conv;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 8]))) / m_conv;
		data->East_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 12]))) / m_conv;
		data->North_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 16]))) / m_conv;
		data->Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 20]))) / m_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
	{
		i = 22;

		data->Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 0]))) / deg_conv_8byte;
		data->Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 8]))) / deg_conv_8byte;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 16]))) / 1000;
		data->East_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 20]))) / 1000000;
		data->North_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 24]))) / 1000000;
		data->Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 28]))) / 1000000;

		j = 119;

		data->GNSS_Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[j + 0]))) / deg_conv_8byte;
		data->GNSS_Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[j + 8]))) / deg_conv_8byte;
		data->GNSS_Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 16]))) / 1000;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 16]))) / 1000;
		data->GNSS_Horizontal_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 20]))) / 1000000;
		data->GNSS_Trackover_Ground = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 24]))) / 1000000;
		data->GNSS_Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[j + 28]))) / 1000000;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_RAWIMU_DATA_RECEIVE:
	{
		i = 60;

		data->Latitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 0]))) / deg_conv_8byte;
		data->Longitude = (double)(*(int64_t*)(&(INSInt->dataBuffer[i + 8]))) / deg_conv_8byte;
		data->Altitude = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 16]))) / 1000;
		data->East_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 20]))) / m_conv;
		data->North_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 24]))) / m_conv;
		data->Vertical_Speed = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 28]))) / m_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		printf("Position data  not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode;

}

IL_ERROR_CODE INS_getQuaternionData(IL_INS* ins, INSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int quat_conv = 10000;
	INSInternal* INSInt;

	//printf("inside INS_getQuaternionData\n");

	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{

	case IL_QPVT_RECEIVE:
	{
		i = 6;

		data->quaternion.Lk0 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) / quat_conv;
		data->quaternion.Lk1 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / quat_conv;
		data->quaternion.Lk2 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4]))) / quat_conv;
		data->quaternion.Lk3 = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 6]))) / quat_conv;

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

IL_ERROR_CODE INS_getPressureBarometricData(IL_INS* ins, INSCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int Hbar_conv = 100;
	int Pbar_conv = 2;
	INSInternal* INSInt;

#if IL_DBG
	printf("inside INS_getPressureBarometricData\n");
# endif
	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT_RECEIVE:
	case IL_QPVT_RECEIVE:
	{
		i = (ins->cmd_flag == IL_OPVT_RECEIVE) ? 91 : 93;


		data->P_bar = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) * Pbar_conv;
		data->H_bar = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 2]))) / Hbar_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT2A_RECEIVE:
	case IL_OPVT2AW_RECEIVE:
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVTAD_RECEIVE:
	{

		i = (ins->cmd_flag == IL_OPVT2A_RECEIVE) ? 100 : (ins->cmd_flag == IL_OPVT2AW_RECEIVE) ? 102 : (ins->cmd_flag == IL_OPVT2AHR_RECEIVE) ? 128 : 124;


		data->P_bar = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0]))) * Pbar_conv;
		data->H_bar = (double)(*(int32_t*)(&(INSInt->dataBuffer[i + 2]))) / Hbar_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		printf("Pressure Barometric data not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode;
}

IL_ERROR_CODE INS_getLatencyData(IL_INS* ins, INSPositionData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	INSInternal* INSInt;
#if IL_DBG
	printf("inside INS_getLatencyData\n");
#endif
	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT_RECEIVE:
	{
		i = 89;

		data->Latency_ms_pos = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 0])));
		data->Latency_ms_vel = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 1])));

		errorCode = ILERR_NO_ERROR;

		break;
	}
	case IL_QPVT_RECEIVE:
	case IL_OPVTAD_RECEIVE:
	case IL_OPVT2AW_RECEIVE:
	{

		i = (ins->cmd_flag == IL_OPVTAD_RECEIVE) ? 117 : 91;

		data->Latency_ms_pos = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 0])));
		data->Latency_ms_vel = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 1])));

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVT2A_RECEIVE:
	{

		i = (ins->cmd_flag == IL_OPVT2A_RECEIVE) ? 94 : 122;

		data->Latency_ms_pos = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 0])));
		data->Latency_ms_vel = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2])));
		data->Latency_ms_vel = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 4])));
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_MINIMAL_DATA_RECEIVE:
	{
		i = 82;

		data->Latency_ms_pos = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 0])));
		data->Latency_ms_vel = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 1])));
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
	{
		i = 189;
		data->Latency_ms_pos = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 0])));
		data->Latency_ms_vel = (double)(*(int8_t*)(&(INSInt->dataBuffer[i + 1])));
		errorCode = ILERR_NO_ERROR;
		break;
	}
	default:
	{
		printf("Latency ms_head, Latency ms_pos, and Latency ms_vel are not supported in this data output format!!! \n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE INS_GNSS_Details(IL_INS* ins, INSGnssData* data)
{

	// only for sensor data format
	IL_ERROR_CODE errorCode;
	int i;
	int gnss_conv = 100;
	INSInternal* INSInt;
#if IL_DBG
	printf("inside INS_GNSS_Details \n");
#endif
	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	if (ins->cmd_flag == IL_SENSOR_DATA_RECEIVE)
	{
		int sol_stat = (int)(*(int8_t*)(&(INSInt->dataBuffer[74])));

		data->sol_stat = sol_stat;

		switch (sol_stat)
		{
		case 0:
			printf("Solution computed");
			break;
		case 1:
			printf("Insufficient observations");
			break;
		case 2:
			printf("No convergence");
			break;
		case 3:
			printf("Singularity at parameters matrix");
			break;
		case 4:
			printf("Covariance trace exceeds maximum (trace > 1000 m)");
			break;
		case 5:
			printf("Test distance exceeded (maximum of 3 rejections if distance >10 km)");
			break;
		case 6:
			printf("Not yet converged from cold start");
			break;
		case 7:
			printf("Height or velocity limits exceeded (in accordance with export licensing restrictions)");
			break;
		case 8:
			printf("Variance exceeds limits");
			break;
		case 9:
			printf("Residuals are too large");
			break;
		case 13:
			printf("Large residuals make position unreliable");
			break;
		case 18:
			printf("When a FIX POSITION command is entered, the receiver computes its own position and determines if the fixed position is valid");
			break;
		case 19:
			printf("The fixed position, entered using the FIX POSITION command, is not valid");
			break;
		case 20:
			printf("Position type is unauthorized - HP or XP on a receiver not authorized");
			break;


		default:
			printf("check the data once , its not in the range !!!! data: %d", sol_stat);
			break;
		}

		int pos_type = (int)(*(int8_t*)(&(INSInt->dataBuffer[75])));

		data->pos_type = pos_type;

		switch (pos_type)
		{
		case 0:
			printf("No solution");
			break;
		case 8:
			printf("Velocity computed using instantaneous Doppler");
			break;
		case 16:
			printf("Single point position");
			break;
		case 17:
			printf("Pseudorange differential solution");
			break;
		case 18:
			printf("Solution calculated using corrections from an WAAS");
			break;
		case 19:
			printf("Propagated by a Kalman filter without new observations");
			break;
		case 20:
			printf("OmniSTAR VBS position");
			break;
		case 32:
			printf("Floating L1 ambiguity solution");
			break;
		case 33:
			printf("Floating L1 ambiguity solution");
			break;
		case 34:
			printf("Floating narrow-lane ambiguity solution");
			break;
		case 48:
			printf("Integer L1 ambiguity solution");
			break;
		case 50:
			printf("nteger narrow-lane ambiguity solution");
			break;
		case 64:
			printf("OmniSTAR HP position");
			break;
		case 65:
			printf("OmniSTAR XP or G2 position");
			break;
		case 68:
			printf("Converging PPP TerraStar-C solution ");
			break;
		case 69:
			printf("Converged PPP TerraStar-C solution (2)");
			break;
		case 77:
			printf("Converging PPP TerraStar-L solution (2)");
			break;
		case 78:
			printf("Converged PPP TerraStar-L solution (2)");
			break;

		default:
			printf("check the data once , its not in the range !!!! data: %d", pos_type);
			break;
		}
		errorCode = ILERR_NO_ERROR;
	}

	return errorCode;
}
IL_ERROR_CODE INS_getGNSS_HeadPitch(IL_INS* ins, INSPositionData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int gnss_conv = 100;
	INSInternal* INSInt;
#if IL_DBG
	printf("inside INS_getGNSS_HeadPitch \n");
#endif
	INSInt = INS_getInternalData(ins);

	if (INSInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (ins->cmd_flag)
	{
	case IL_OPVT2AW_RECEIVE:
	{

		i = 94;

		data->GNSS_Heading = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / gnss_conv;
		data->GNSS_Pitch = (double)(*(int16_t*)(&(INSInt->dataBuffer[i + 2]))) / gnss_conv;
		data->GNSS_Heading_STD = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 4]))) / gnss_conv;
		data->GNSS_Pitch_STD = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 6]))) / gnss_conv;

		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVTAD_RECEIVE:
	case IL_OPVT2AHR_RECEIVE:
	case IL_OPVT2A_RECEIVE:
	{
		i = (ins->cmd_flag == IL_OPVT2A_RECEIVE) ? 92 : 120;

		data->GNSS_Heading = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / gnss_conv;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
	{
		i = 152;

		data->GNSS_Heading = (double)(*(uint16_t*)(&(INSInt->dataBuffer[i + 0]))) / gnss_conv;
		errorCode = ILERR_NO_ERROR;
		break;

	}
	default:
	{
		printf("Heading GNSS or Pitch GNSS or Heading STD GNSS or Pitch STD GNSS not supported for this data output format\n");
		errorCode = ILERR_NO_ERROR;
		break;
	}
	}

	return errorCode;
}
IL_ERROR_CODE INS_SetMode(IL_INS* ins, int mode)
{
	ins->mode = mode;
	unsigned int syncDataOutputType = 100;
	IL_BOOL waitForResponse = 1;
	if (ins->mode)
	{
		ins->cmd_flag = IL_SET_ONREQUEST_CMD;
		INS_setSetOnRequestMode(ins, syncDataOutputType, waitForResponse);
		return ILERR_NO_ERROR;
	}
	else
	{
#if IL_DBG
		printf("now you can call datamode functions you want to recive from the INS \n ");
#endif
#if defined(WIN32) || defined(WIN64) 
		Sleep(2000);
#else
		sleep(2);
#endif 


		return ILERR_NO_ERROR;
	}
}

IL_ERROR_CODE INS_Stop(IL_INS* ins)
{
	int errorCode;
#if IL_DBG
	printf("inside INS_Stop \n");
#endif
	unsigned char stop_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xFE ,0x05, 0x01 };

	//printf("size of stop %d \n" , sizeof(stop_payload));
	ins->cmd_flag = IL_STOP_CMD;
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, stop_payload, sizeof(stop_payload));
	return errorCode;
}

IL_ERROR_CODE INS_setSetOnRequestMode(IL_INS* ins, unsigned int syncDataOutputType, IL_BOOL waitForResponse)
{

	int errorCode;
	unsigned char  setonrequest_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xC1 ,0xC8 ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;

#if IL_DBG
	printf("inside INS_setSetOnRequestMode\n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, setonrequest_payload, sizeof(setonrequest_payload));
#if IL_DBG
	printf("inside INS_setSetOnRequestMode done\n");
#endif
	return errorCode;
}

IL_ERROR_CODE INS_OPVTdata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  OPVTdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x52 ,0x59 ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVTdata_Recive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVTdata_payload, sizeof(OPVTdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_QPVTdata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  QPVTdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x56 ,0x5D ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_QPVTdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, QPVTdata_payload, sizeof(QPVTdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVT2Adata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  OPVT2Adata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x57 ,0x5E ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVT2Adata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVT2Adata_payload, sizeof(OPVT2Adata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVT2AWdata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  OPVT2AWdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x59 ,0x60 ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVT2AWdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVT2AWdata_payload, sizeof(OPVT2AWdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVT2AHRdata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  OPVT2AHRdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x58 ,0x5F,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVT2AHRdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVT2AHRdata_payload, sizeof(OPVT2AHRdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVTADdata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  OPVTADdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x61 ,0x68 ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVTADdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVTADdata_payload, sizeof(OPVTADdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_SensorsData_Receive(IL_INS* ins)
{
	int errorCode;

	unsigned char  SensorsData_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x50 ,0x57 ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_SensorsData_Receive \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, SensorsData_payload, sizeof(SensorsData_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVT_rawIMUdata_Receive(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 66 6D 00
	unsigned char  OPVT_rawIMUdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x66 ,0x6D ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVT_rawIMUdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVT_rawIMUdata_payload, sizeof(OPVT_rawIMUdata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_OPVT_GNSSextdata_Receive(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 67 6E 00
	unsigned char  OPVT_GNSSextdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x67 ,0x6E ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_OPVT_GNSSextdata_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, OPVT_GNSSextdata_payload, sizeof(OPVT_GNSSextdata_payload));

	return errorCode;
}


IL_ERROR_CODE SPAN_rawimu_Receive(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 68 6F 00
	unsigned char  SPAN_rawimu_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x68 ,0x6F ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside SPAN_rawimu_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, SPAN_rawimu_payload, sizeof(SPAN_rawimu_payload));

	return errorCode;
}

IL_ERROR_CODE UserDef_Data_Receive(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 95 9C 00
	unsigned char  UserDef_Data_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x95 ,0x9C ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside UserDef_Data_Receive \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, UserDef_Data_payload, sizeof(UserDef_Data_payload));

	return errorCode;
}
//**
IL_ERROR_CODE UserDef_DataConfig_Receive(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 96 9D 00
	unsigned char  UserDef_DataConfig_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x96 ,0x9D ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside UserDef_DataConfig_Receive \n");
#endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, UserDef_DataConfig_payload, sizeof(UserDef_DataConfig_payload));

	return errorCode;
}
//**
IL_ERROR_CODE Get_UserDef_DataStruct(IL_INS* ins)
{
	int errorCode;

	//AA 55 00 00 07 00 97 9E 00
	unsigned char  Get_UserDef_DataStruct_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x97 ,0x9E ,0x00 };


	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside Get_UserDef_DataStruct \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, Get_UserDef_DataStruct_payload, sizeof(Get_UserDef_DataStruct_payload));

	return errorCode;
}


IL_ERROR_CODE INS_Minimaldata_Receive(IL_INS* ins)
{
	int errorCode;
	unsigned char  Minimaldata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x53 ,0x5A ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_Minimaldata_Receive \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, Minimaldata_payload, sizeof(Minimaldata_payload));

	return errorCode;
}

IL_ERROR_CODE INS_NMEA_Receive(IL_INS* ins)
{
	int errorCode;
	//AA 55 00 00 07 00 54 5B 00
	unsigned char  NMEA_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x54 ,0x5B ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_NMEA_Receive \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, NMEA_payload, sizeof(NMEA_payload));

	return errorCode;
}

IL_ERROR_CODE INS_Sensors_NMEA_Receive(IL_INS* ins)
{
	int errorCode;
	//AA 55 00 00 07 00 55 5C 00
	unsigned char  Sensors_NMEA_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x55 ,0x5C ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_Sensors_NMEA_Receive \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, Sensors_NMEA_payload, sizeof(Sensors_NMEA_payload));

	return errorCode;
}


IL_ERROR_CODE INS_DevSelfTest(IL_INS* ins)
{
	int errorCode;
	unsigned char  DevSelfTest_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x13 ,0x1A ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_DevSelfTest \n");
# endif
	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, DevSelfTest_payload, sizeof(DevSelfTest_payload));

	return errorCode;
}

IL_ERROR_CODE INS_ReadINSpar(IL_INS* ins)
{
	int errorCode;
	unsigned char  ReadINSpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x41 ,0x48 ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_ReadINSpar \n");
#endif
	ins->cmd_flag = IL_READ_INS_PAR_RECEIVE;

	INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, ReadINSpar_payload, sizeof(ReadINSpar_payload));


	return errorCode;
}

IL_ERROR_CODE INS_LoadINSpar(IL_INS* ins)
{
	int errorCode;

	unsigned char  LoadINSpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x40 ,0x47 ,0x00 };

	if (!ins->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside INS_LoadINSpar \n");
# endif
	ins->cmd_flag = IL_LOAD_INS_PAR_RECEIVE;

	//INS_Recive_size(ins);
	errorCode = INS_writeOutCommand(ins, LoadINSpar_payload, sizeof(LoadINSpar_payload));

	return errorCode;
}



void INS_Recive_size(IL_INS* ins)
{
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);
#if IL_DBG
	printf("now ins->cmd_flag : %d", ins->cmd_flag);
# endif
	switch (ins->cmd_flag)
	{
	case IL_STOP_CMD:
		INSInt->num_bytes_recive = IL_STOP_CMD_RECEIVE_SIZE;
		break;
	case IL_SET_ONREQUEST_CMD:
		INSInt->num_bytes_recive = IL_SET_ONREQUEST_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT_CMD_RECEIVE_SIZE;
		break;
	case IL_QPVT_RECEIVE:
		INSInt->num_bytes_recive = IL_QPVT_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT2A_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT2A_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT2AW_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT2AW_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT2AHR_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT2AHR_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVTAD_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVTAD_CMD_RECEIVE_SIZE;
		break;
	case IL_MINIMAL_DATA_RECEIVE:
		INSInt->num_bytes_recive = IL_MINIMAL_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_DEV_SELF_TEST_RECEIVE:
		INSInt->num_bytes_recive = IL_DEV_SELF_TEST_CMD_RECEIVE_SIZE;
		break;
	case IL_READ_INS_PAR_RECEIVE:
		INSInt->num_bytes_recive = IL_READ_INS_PAR_CMD_RECEIVE_SIZE;
		break;
	case IL_LOAD_INS_PAR_RECEIVE:
		INSInt->num_bytes_recive = IL_LOAD_INS_PAR_CMD_RECEIVE_SIZE;
		break;
	case IL_SENSOR_DATA_RECEIVE:
		INSInt->num_bytes_recive = IL_SENSOR_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT_RAWIMU_DATA_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT_RAWIMU_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_OPVT_GNSSEXT_DATA_RECEIVE:
		INSInt->num_bytes_recive = IL_OPVT_GNSSEXT_DATA_CMD_RECEIVE_SIZE;
		break;
	default:
		break;
	}
}

IL_BOOL INS_shouldCheckForResponse_threadSafe(IL_INS* ins, char* responseMatchBuffer, int num_bytes_to_recive)
{
	INSInternal* INSInt;
	IL_BOOL shouldCheckResponse;

	INSInt = INS_getInternalData(ins);

	inertial_criticalSection_enter(&INSInt->critSecForResponseMatchAccess);
	shouldCheckResponse = 1; // to avoid warning now
	inertial_criticalSection_leave(&INSInt->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}


void INS_disableResponseChecking_threadSafe(IL_INS* ins)
{
	INSInternal* INSInt;

	INSInt = INS_getInternalData(ins);

	inertial_criticalSection_enter(&INSInt->critSecForResponseMatchAccess);

	INSInt->checkForResponse = IL_FALSE;
	INSInt->cmdResponseSize = 0;
	INSInt->cmdResponseMatchBuffer[0] = 0;

	inertial_criticalSection_leave(&INSInt->critSecForResponseMatchAccess);
}


IL_ERROR_CODE INS_registerDataReceivedListener(IL_INS* ins, INSNewDataReceivedListener listener)
{
	INSInternal* INSInt = INS_getInternalData(ins);

	if (INSInt->DataListener != NULL)
		return ILERR_UNKNOWN_ERROR;

	INSInt->DataListener = listener;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE INS_unregisterDataReceivedListener(IL_INS* ins, INSNewDataReceivedListener listener)
{
	INSInternal* INSInt = INS_getInternalData(ins);

	if (INSInt->DataListener == NULL)
		return ILERR_UNKNOWN_ERROR;

	if (INSInt->DataListener != listener)
		return ILERR_UNKNOWN_ERROR;

	INSInt->DataListener = NULL;

	return ILERR_NO_ERROR;
}

unsigned char INS_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char)cmdToCheck[i];

	return xorVal;
}

void INS_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = INS_checksum_compute(cmdToCheck);
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

void INS_processAsyncData(IL_INS* ins, unsigned char buffer[])
{
	INSCompositeData data;
	INSInternal* INSInt = INS_getInternalData(ins);

	memset(&data, 0, sizeof(INSCompositeData));


	/* We had an async data packet and need to move it to INSInt->lastestAsyncData. */
	inertial_criticalSection_enter(&INSInt->critSecForLatestAsyncDataAccess);
	memcpy(&INSInt->lastestAsyncData, &data, sizeof(INSCompositeData));
	inertial_criticalSection_leave(&INSInt->critSecForLatestAsyncDataAccess);

	if (INSInt->DataListener != NULL)
		INSInt->DataListener(ins, &INSInt->lastestAsyncData);


}




/** \endcond */