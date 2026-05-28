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
#include "InertialLabs_IMU.h"
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
	 * be checking to a command response comming from the IMU device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the IMU. This field should only be accessed in the
	 * functions IMU_shouldCheckForResponse_threadSafe,
	 * IMU_enableResponseChecking_threadSafe, and
	 * IMU_disableResponseChecking_threadSafe.
	 */
	IL_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the IMU is a match for the command sent.
	 * This field should only be accessed in the functions
	 * IMU_shouldCheckForResponse_threadSafe, IMU_enableResponseChecking_threadSafe,
	 * and IMU_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[IL_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the IMU device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "IMU_ClbData" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[IL_MAX_RESPONSE_SIZE + 1];

	unsigned char*       	dataBuffer;
	//unsigned char           dataBuffer[READ_BUFFER_SIZE];
	int 			        cmdResponseSize;

	int 					num_bytes_recive;
	int 					recive_flag;

	IMUCompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	IMUNewDataReceivedListener DataListener;

} IMUInternal;

/* Private type definitions. *************************************************/


void* IMU_communicationHandler(void*);

IL_ERROR_CODE  IMU_writeOutCommand(IL_IMU* imu, unsigned char cmdToSend[], int cmdSize);


/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	imu	Pointer to the IL_IMU control object.
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
IL_BOOL IMU_shouldCheckForResponse_threadSafe(IL_IMU* imu, char* responseMatchBuffer, int num_bytes_to_recive);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	imu	Pointer to the IL_IMU control object.
 *
 * \param[in]	responseMatch
 * \\ TODO: add the comments here
 */
void IMU_enableResponseChecking_threadSafe(IL_IMU* imu, int responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	imu 	Pointer to the IL_IMU control object.
 */
void IMU_disableResponseChecking_threadSafe(IL_IMU* imu);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command  and transmits it to the device.
 * The function will then wait until the response is received. The response
 * will be located in the IMUInternal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	imu				Pointer to the IL_IMU control object.
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
IL_ERROR_CODE IMU_transaction(IL_IMU* imu, unsigned char cmdToSend[], int responseMatch);


/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions  inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int IMU_writeData_threadSafe(IL_IMU* imu, unsigned char dataToSend[], unsigned int dataLength);


/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions inertial_writeData_threadSafe and inertial_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
IL_ERROR_CODE IMU_readData_threadSafe(IL_IMU* imu, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a IMU control object.
 *
 * \param[in]	imu	Pointer to the IMU control object.
 * \return The internal data.
 */
IMUInternal* IMU_getInternalData(IL_IMU* imu);

void IMU_processAsyncData(IL_IMU* imu, unsigned char buffer[]);

/**
 * \brief  Process and Check the received packaet and transfer in to the internal structure.
 *
 * \param[in]	imu	Pointer to the IMU control object.
 * \param[in]   data buffer
 * \param[in]   Number of bytes recieved from the IMU
 *
 * \return Copy the data buffer to internal data structure variable.
 */
void IMU_processReceivedPacket(IL_IMU* imu, unsigned char buffer[], int num_bytes_to_recive);

/**
 * \brief Helper method to set the buffer size in the IMU control object,according to the command.
 *
 * \param[in]	imu	Pointer to the IMU control object.
 * \return buffer size in  internal data structure.
 */
void IMU_Recive_size(IL_IMU* imu);


/* Function definitions. *****************************************************/


IL_ERROR_CODE IMU_connect(IL_IMU* newIMU, const char* portName, int baudrate)
{

	IMUInternal* IMUInt;
	IL_ERROR_CODE errorCode;

	/* Allocate memory. */
	IMUInt = (IMUInternal*)malloc(sizeof(IMUInternal));
	newIMU->internalData = IMUInt;

	newIMU->portName = (char*)malloc(strlen(portName) + 1);
	strcpy(newIMU->portName, portName);
	newIMU->baudRate = baudrate;
	newIMU->isConnected = IL_FALSE;
	IMUInt->continueServicingComPort = IL_TRUE;
	IMUInt->checkForResponse = IL_FALSE;
	IMUInt->timeout = DEFAULT_TIMEOUT_IN_MS;
	IMUInt->DataListener = NULL;
	IMUInt->recive_flag = ILERR_UNKNOWN_ERROR;

	memset(&IMUInt->lastestAsyncData, 0, sizeof(IMUCompositeData));


	errorCode = inertial_comPort_open(&IMUInt->comPortHandle, portName, baudrate);

	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_criticalSection_initialize(&IMUInt->criticalSection);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&IMUInt->critSecForComPort);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&IMUInt->critSecForResponseMatchAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_criticalSection_initialize(&IMUInt->critSecForLatestAsyncDataAccess);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_event_create(&IMUInt->waitForThreadToStopServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&IMUInt->waitForCommandResponseEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;
	errorCode = inertial_event_create(&IMUInt->waitForThreadToStartServicingComPortEvent);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	errorCode = inertial_thread_startNew(&IMUInt->comPortServiceThreadHandle, &IMU_communicationHandler, newIMU);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;

	newIMU->isConnected = IL_TRUE;

	errorCode = inertial_event_waitFor(IMUInt->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != ILERR_NO_ERROR)
		return errorCode;


	return ILERR_NO_ERROR;
}


IL_ERROR_CODE IMU_disconnect(IL_IMU* imu)
{
	IMUInternal* IMUInt;

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;

	IMUInt = IMU_getInternalData(imu);

	IMUInt->continueServicingComPort = IL_FALSE;

	inertial_event_waitFor(IMUInt->waitForThreadToStopServicingComPortEvent, -1);
	IMU_Stop(imu);
	//sleep(3);

	inertial_comPort_close(IMUInt->comPortHandle);

	inertial_criticalSection_dispose(&IMUInt->criticalSection);
	inertial_criticalSection_dispose(&IMUInt->critSecForComPort);
	inertial_criticalSection_dispose(&IMUInt->critSecForResponseMatchAccess);
	inertial_criticalSection_dispose(&IMUInt->critSecForLatestAsyncDataAccess);


	/* Free the memory associated with the IMU structure. */
	//free(&IMUInt->waitForThreadToStopServicingComPortEvent);
	//free(&IMUInt->waitForCommandResponseEvent);
	//free(&IMUInt->waitForThreadToStartServicingComPortEvent);
	free(imu->internalData);

	imu->isConnected = IL_FALSE;
#if defined(WIN32) || defined(WIN64) 
	Sleep(3000);
#else
	sleep(3);
#endif 

	return ILERR_NO_ERROR;
}


int IMU_writeData_threadSafe(IL_IMU* imu, unsigned char dataToSend[], unsigned int dataLength)
{
	int errorCode;
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

#if IL_DBG
	printf("IMU_writeData_threadSafe : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n", dataToSend[0], dataToSend[1], dataToSend[2],
		dataToSend[3], dataToSend[4], dataToSend[5],
		dataToSend[6], dataToSend[7], dataToSend[8]);

#endif 
	inertial_criticalSection_enter(&IMUInt->critSecForComPort);
	errorCode = inertial_comPort_writeData(IMUInt->comPortHandle, dataToSend, dataLength);
	inertial_criticalSection_leave(&IMUInt->critSecForComPort);

	//printf("after read");

	return errorCode;
}

IL_ERROR_CODE IMU_readData_threadSafe(IL_IMU* imu, unsigned char dataBuffer[], unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	IL_ERROR_CODE errorCode;
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

	inertial_criticalSection_enter(&IMUInt->critSecForComPort);
	errorCode = inertial_comPort_readData(IMUInt->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	inertial_criticalSection_leave(&IMUInt->critSecForComPort);

	if (imu->mode)
	{

		if ((imu->cmd_flag == IL_IMU_CLB_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_IMU_CLB_DATA_CMD_RECEIVE_SIZE) ||
			(imu->cmd_flag == IL_IMU_GA_DATA_RECEIVE && *numOfBytesActuallyRead == (int)IL_IMU_GA_DATA_CMD_RECEIVE_SIZE) ||
			(imu->cmd_flag == IL_IMU_ORIENTATION_RECEIVE && *numOfBytesActuallyRead == (int)IL_IMU_ORIENTATION_CMD_RECEIVE_SIZE) ||
			(imu->cmd_flag == IL_IMU_PSTABILIZATION_RECEIVE && *numOfBytesActuallyRead == (int)IL_IMU_PSTABILIZATION_CMD_RECEIVE_SIZE) ||
			(imu->cmd_flag == IL_IMU_GET_DEV_INFO_RECEIVE && *numOfBytesActuallyRead == (int)IL_IMU_GET_DEV_INFO_CMD_RECEIVE_SIZE)) 
		{
			return ILERR_NO_ERROR;
		}
		else if (imu->cmd_flag == IL_STOP_CMD && *numOfBytesActuallyRead == IL_STOP_CMD_RECEIVE_SIZE)
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

IL_ERROR_CODE IMU_writeOutCommand(IL_IMU* imu, unsigned char cmdToSend[], int cmdSize)
{
	//char packetTail[] = {0xFF ,0xFF};
	 //printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  %d \n", cmdToSend[0] ,cmdToSend[1] ,cmdToSend[2],
	 //																				cmdToSend[3] ,cmdToSend[4],cmdToSend[5],
	 //																				cmdToSend[6] ,cmdToSend[7],cmdToSend[8], cmdSize);

	 //printf("size of : %d \n ", strlen(cmdToSend));
	IMU_writeData_threadSafe(imu, cmdToSend, cmdSize);

	//
	//IMU_writeData_threadSafe(imu,packetTail, strlen(packetTail));

	return ILERR_NO_ERROR;
}


void* IMU_communicationHandler(void* IMUobj)
{

	IL_IMU* imu;
	IMUInternal* IMUInt;
	unsigned char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	unsigned char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	IL_BOOL haveFoundStartOfCommand = IL_FALSE;

	memset(readBuffer, 0x00, sizeof(unsigned char) * READ_BUFFER_SIZE);

	imu = (IL_IMU*)IMUobj;
	IMUInt = IMU_getInternalData(imu);

	inertial_event_signal(IMUInt->waitForThreadToStartServicingComPortEvent);

	while (IMUInt->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		IMU_readData_threadSafe(imu, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);


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
			printf("number of bytes to recive : %d \n", IMUInt->num_bytes_recive);
			printf(" cmd_flag : %d \n", imu->cmd_flag);
#endif 

			switch (imu->cmd_flag)
			{
			case IL_STOP_CMD:
#if IL_DBG
				printf("IL_STOP_CMD \n");
#endif
				break;

			case IL_SET_ONREQUEST_CMD:
#if IL_DBG
				printf("IL_SET_ONREQUEST_CMD \n");
#endif
				break;

			case IL_IMU_CLB_DATA_RECEIVE:
			case IL_IMU_GA_DATA_RECEIVE:
			case IL_IMU_ORIENTATION_RECEIVE:
			case IL_IMU_PSTABILIZATION_RECEIVE:
			case IL_IMU_GET_DEV_INFO_RECEIVE:
			{
				if (imu->mode)
				{

					//inertial_criticalSection_enter(&IMUInt->critSecForResponseMatchAccess);
					IMU_processReceivedPacket(imu, readBuffer, IMUInt->num_bytes_recive);
					//inertial_criticalSection_leave(&IMUInt->critSecForResponseMatchAccess);
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
							//free(IMUInt->dataBuffer);
							haveFoundStartOfCommand = IL_TRUE;
							responseBuilderBufferPos = 0;

						}
						if (((int)readBuffer[curResponsePos] == 170) && ((int)readBuffer[curResponsePos + 1] == 68) && ((int)readBuffer[curResponsePos + 2] == 18) ) {
#if IL_DBG
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
							if (responseBuilderBufferPos == IMUInt->num_bytes_recive)
							{
								haveFoundStartOfCommand = IL_FALSE;
							}
						}

						if (responseBuilderBufferPos == IMUInt->num_bytes_recive)
						{
							//inertial_criticalSection_enter(&IMUInt->critSecForResponseMatchAccess);
							IMU_processReceivedPacket(imu, responseBuilderBuffer, IMUInt->num_bytes_recive);
							responseBuilderBufferPos = 0;
							haveFoundStartOfCommand = IL_FALSE;
							//inertial_criticalSection_leave(&IMUInt->critSecForResponseMatchAccess);


						}
						else {
							/* We received data but we have not found the starting point of a command. */

						}
					}
				}

				break;
			}
			case IL_READ_IMU_PAR_RECEIVE:
			{
				IMU_processReceivedPacket(imu, readBuffer, IMUInt->num_bytes_recive);
			}
			case IL_IMU_NMEA_RECEIVE:
			{
				if (imu->mode)
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
				printf("No CMD flag Set !!!! \n");
#endif
				break;
			}

		}
	}
	inertial_event_signal(IMUInt->waitForThreadToStopServicingComPortEvent);

	return IL_NULL;

}


IL_BOOL IMU_verifyConnectivity(IL_IMU* imu)
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


IL_ERROR_CODE IMU_ReadInternalParameters(IL_IMU* imu, IMUSetInternalData* data)
{

	int i = 6;

	int m_conv = 100;
	int deg_conv = 10000000;

	IMUInternal* IMUInt;
	IMUInt = IMU_getInternalData(imu);

	if (IMUInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	//printf("0x%04x", *(uint16_t*)(&(IMUInt->dataBuffer[i+0])));

	data->Data_rate = (double)(*(uint16_t*)(&(IMUInt->dataBuffer[i + 0])));

	data->Initial_Alignment_Time = (double)(*(uint16_t*)(&(IMUInt->dataBuffer[i + 2])));

	data->Magnetic_Declination = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 4]))) / m_conv;

	data->Latitude = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 8]))) / deg_conv;

	data->Longitude = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 12]))) / deg_conv;

	data->Altitude = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 16]))) / m_conv;

	data->Year = (int)(*(int8_t*)(&(IMUInt->dataBuffer[i + 20])));

	data->Month = (int)(*(int8_t*)(&(IMUInt->dataBuffer[i + 21])));

	data->Day = (int)(*(int8_t*)(&(IMUInt->dataBuffer[i + 22])));

	data->Alignment_Angle_A1 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 23]))) / m_conv;

	data->Alignment_Angle_A2 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 25]))) / m_conv;

	data->Alignment_Angle_A3 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 27]))) / m_conv;

	data->IMU_Mount_Right = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 29]))) / m_conv;

	data->IMU_Mount_Forward = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 31]))) / m_conv;

	data->IMU_Mount_Up = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 33]))) / m_conv;

	data->Antenna_Pos_Right = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 35]))) / m_conv;

	data->Antenna_Pos_Forward = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 37]))) / m_conv;

	data->Antenna_Pos_Up = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 39]))) / m_conv;


	data->Altitude_one_byte = (int)(*(int8_t*)(&(IMUInt->dataBuffer[i + 41])));


	data->Baro_Altimeter = (int)(*(int8_t*)(&(IMUInt->dataBuffer[i + 58])));


	//ROS_INFO("\nIMU Device Name: %s\n",data.IMU_Device_Name);

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
		"\n IMU_Mount_Right : %f\n"
		"  IMU_Mount_Forward : %f\n"
		"  IMU_Mount_Up  : %f\n",
		data->IMU_Mount_Right,
		data->IMU_Mount_Forward,
		data->IMU_Mount_Up);

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

	return ILERR_NO_ERROR;

}


IMUInternal* IMU_getInternalData(IL_IMU* imu)
{
	return (IMUInternal*)imu->internalData;
}

IL_ERROR_CODE IMU_transaction(IL_IMU* imu, unsigned char *cmdToSend, int responseMatch)
{
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);


	IMU_enableResponseChecking_threadSafe(imu, responseMatch);

	IMU_writeData_threadSafe(imu, cmdToSend, sizeof(cmdToSend));

	return inertial_event_waitFor(IMUInt->waitForCommandResponseEvent, IMUInt->timeout);
}

void IMU_enableResponseChecking_threadSafe(IL_IMU* imu, int responseMatch)
{
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

	inertial_criticalSection_enter(&IMUInt->critSecForResponseMatchAccess);

	IMUInt->checkForResponse = IL_TRUE;
	IMUInt->cmdResponseSize = responseMatch;

	inertial_criticalSection_leave(&IMUInt->critSecForResponseMatchAccess);
}


void IMU_processReceivedPacket(IL_IMU* imu, unsigned char buffer[], int num_bytes_to_recive)
{
	IMUInternal* IMUInt;


	IMUInt = IMU_getInternalData(imu);

	inertial_criticalSection_enter(&IMUInt->critSecForLatestAsyncDataAccess);
	IMUInt->dataBuffer = buffer;
	IMUInt->recive_flag = ILERR_DATA_IN_BUFFER;
	inertial_criticalSection_leave(&IMUInt->critSecForLatestAsyncDataAccess);

#if  IL_RAW_DATA
	for (int i = 0; i < IMUInt->num_bytes_recive; i++)
	{
		printf("0x%02x ", IMUInt->dataBuffer[i]);
	}
	printf("\n");

#endif

#if 0
	/* See if we should be checking for a command response. */
	if (IMU_shouldCheckForResponse_threadSafe(imu, buffer, num_bytes_to_recive)) {

		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		//if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			 //IMU_disableResponseChecking_threadSafe(imu);

			 /* The line below should be thread-safe since the user thread should be
			  * blocked until we signal that we have received the response. */


			  /* Signal to the user thread we have received a response. */
		inertial_event_signal(IMUInt->waitForCommandResponseEvent);
		//}
	}
#endif 

}


IL_ERROR_CODE IMU_YPR(IL_IMU* imu, IMUCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	IMUInternal* IMUInt;

#if IL_DBG
	printf("inside IMU_YPR\n");

# endif
	IMUInt = IMU_getInternalData(imu);

	if (IMUInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (imu->cmd_flag)
	{
	case IL_IMU_ORIENTATION_RECEIVE:
	case IL_IMU_PSTABILIZATION_RECEIVE:
	{

		i = (imu->cmd_flag == IL_IMU_PSTABILIZATION_RECEIVE) ? 18 : 6;


		data->ypr.yaw = (double)(*(uint16_t*)(&(IMUInt->dataBuffer[i + 0]))) / 100;
		data->ypr.pitch = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) / 100;
		data->ypr.roll = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 4]))) / 100;

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


IL_ERROR_CODE IMU_getGyroAccMag(IL_IMU* imu, IMUCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	int kg = 50;
	int ka = 4000;
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

	if (IMUInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (imu->cmd_flag)
	{
	case IL_IMU_CLB_DATA_RECEIVE:
	case IL_IMU_ORIENTATION_RECEIVE:
	{

	
		i = (imu->cmd_flag == IL_IMU_ORIENTATION_RECEIVE) ? 12 : 6;

		data->gyro.c0 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 0]))) / kg;
		data->gyro.c1 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) / kg;
		data->gyro.c2 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 4]))) / kg;


		i = i + 6;

		data->acceleration.c0 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 0]))) / ka;
		data->acceleration.c1 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) / ka;
		data->acceleration.c2 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 4]))) / ka;

		i = i + 6;

		data->magnetic.c0 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 0]))) * 10;
		data->magnetic.c1 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) * 10;
		data->magnetic.c2 = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 4]))) * 10;

		errorCode = ILERR_NO_ERROR;

		break;
	}
	case IL_IMU_GA_DATA_RECEIVE:
	{
		
		i = 6 ;
		data->gyro.c0 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 0]))) / 100000;
		data->gyro.c1 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 4]))) / 100000;
		data->gyro.c2 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 8]))) / 100000;

		i = 18;

		data->acceleration.c0 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 0]))) / 1000000;
		data->acceleration.c1 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 4]))) / 1000000;
		data->acceleration.c2 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 8]))) / 1000000;


		data->magnetic.c0 = 0.0;
		data->magnetic.c1 = 0.0;
		data->magnetic.c2 = 0.0;

		errorCode = ILERR_NO_ERROR;
		break;
	
	}
	case IL_IMU_PSTABILIZATION_RECEIVE:
	{
		i = 6 ;
		data->gyro.c0 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 0]))) / 100000;
		data->gyro.c1 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 4]))) / 100000;
		data->gyro.c2 = (double)(*(int32_t*)(&(IMUInt->dataBuffer[i + 8]))) / 100000;

		data->magnetic.c0 = 0.0;
		data->magnetic.c1 = 0.0;
		data->magnetic.c2 = 0.0;

		data->acceleration.c0 = 0;
		data->acceleration.c1 = 0;
		data->acceleration.c2 = 0;

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

IL_ERROR_CODE IMU_getSensorData(IL_IMU* imu, IMUCompositeData* data)
{
	IL_ERROR_CODE errorCode;
	int i;
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

	if (IMUInt->recive_flag != ILERR_DATA_IN_BUFFER)
		return ILERR_MEMORY_ERROR;

	switch (imu->cmd_flag)
	{
	case IL_IMU_CLB_DATA_RECEIVE:
	case IL_IMU_GA_DATA_RECEIVE:
	case IL_IMU_ORIENTATION_RECEIVE:
	{

		i = (imu->cmd_flag == IL_IMU_GA_DATA_RECEIVE) ? 34 : (imu->cmd_flag == IL_IMU_ORIENTATION_RECEIVE) ? 36 : 32;

		data->Vinp = (double)(*(uint16_t*)(&(IMUInt->dataBuffer[i + 0]))) / 100;
		data->Temper = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) / 10;
		errorCode = ILERR_NO_ERROR;
		break;
	}
	case IL_IMU_PSTABILIZATION_RECEIVE:
	{
		i = 24;
		data->Temper = (double)(*(int16_t*)(&(IMUInt->dataBuffer[i + 2]))) / 10;
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

IL_ERROR_CODE IMU_SetMode(IL_IMU* imu, int mode)
{
	imu->mode = mode;
	unsigned int syncDataOutputType = 100;
	IL_BOOL waitForResponse = 1;
	if (imu->mode)
	{
		imu->cmd_flag = IL_SET_ONREQUEST_CMD;
		IMU_setSetOnRequestMode(imu, syncDataOutputType, waitForResponse);
		return ILERR_NO_ERROR;
	}
	else
	{
		printf("now you can call datamode functions you want to recive from the IMU ");
#if defined(WIN32) || defined(WIN64) 
		Sleep(2000);
#else
		sleep(2);
#endif 


		return ILERR_NO_ERROR;
	}
}


IL_ERROR_CODE IMU_Stop(IL_IMU* imu)
{
	int errorCode;
#if IL_DBG
	printf("Stopping the IMU device \n");
# endif
	unsigned char stop_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xFE ,0x05, 0x01 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;

	//printf("size of stop %d \n" , sizeof(stop_payload));
	imu->cmd_flag = IL_STOP_CMD;
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, stop_payload, sizeof(stop_payload));
	return errorCode;
}

IL_ERROR_CODE IMU_setSetOnRequestMode(IL_IMU* imu, unsigned int syncDataOutputType, IL_BOOL waitForResponse)
{

	int errorCode;
	unsigned char  setonrequest_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0xC1 ,0xC8 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_setSetOnRequestMode\n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, setonrequest_payload, sizeof(setonrequest_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_ClbData_Receive(IL_IMU* imu)
{
	int errorCode;
	unsigned char  ClbData_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x8D ,0x94 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_ClbData_Receive \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, ClbData_payload, sizeof(ClbData_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_GAdata_Receive(IL_IMU* imu)
{
	int errorCode;
	unsigned char  GAdata_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x8F ,0x96 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_GAdata_Receive \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, GAdata_payload, sizeof(GAdata_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_Orientation_Receive(IL_IMU* imu)
{
	int errorCode;
	unsigned char  Orientation_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x33 ,0x3A ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_Orientation_Receive \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, Orientation_payload, sizeof(Orientation_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_PStabilization_Receive(IL_IMU* imu)
{
	int errorCode;
	unsigned char  PStabilization_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x92 ,0x99 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_PStabilization_Receive \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, PStabilization_payload, sizeof(PStabilization_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_NMEA_Receive(IL_IMU* imu)
{
	int errorCode;
	unsigned char  NMEA_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x8E ,0x95 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_NMEA_Receive \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, NMEA_payload, sizeof(NMEA_payload));

	return errorCode;
}


IL_ERROR_CODE ReadIMUpar(IL_IMU* imu)
{
	int errorCode;
	unsigned char  ReadIMUpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x41 ,0x48 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside ReadIMUpar \n");
# endif
	imu->cmd_flag = IL_READ_IMU_PAR_RECEIVE;

	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, ReadIMUpar_payload, sizeof(ReadIMUpar_payload));


	return errorCode;
}

IL_ERROR_CODE LoadIMUpar(IL_IMU* imu)
{
	int errorCode;

	unsigned char  LoadIMUpar_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x40 ,0x47 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside LoadIMUpar \n");
# endif
	imu->cmd_flag = IL_LOAD_IMU_PAR_RECEIVE;

	//IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, LoadIMUpar_payload, sizeof(LoadIMUpar_payload));

	return errorCode;
}

IL_ERROR_CODE IMU_GetDevInfo(IL_IMU* imu)
{
	int errorCode;

	unsigned char  GetDevInfo_payload[] = { 0XAA, 0x55, 0x00, 0x00, 0x07,0x00,0x12 ,0x19 ,0x00 };

	if (!imu->isConnected)
		return ILERR_NOT_CONNECTED;
#if IL_DBG
	printf("inside IMU_GetDevInfo \n");
# endif
	IMU_Recive_size(imu);
	errorCode = IMU_writeOutCommand(imu, GetDevInfo_payload, sizeof(GetDevInfo_payload));

	return errorCode;
}





void IMU_Recive_size(IL_IMU* imu)
{
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);
#if IL_DBG
	printf("now imu->cmd_flag : %d", imu->cmd_flag);
#endif
	switch (imu->cmd_flag)
	{
	case IL_STOP_CMD:
		IMUInt->num_bytes_recive = IL_STOP_CMD_RECEIVE_SIZE;
		break;
	case IL_SET_ONREQUEST_CMD:
		IMUInt->num_bytes_recive = IL_SET_ONREQUEST_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_CLB_DATA_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_CLB_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_GA_DATA_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_GA_DATA_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_ORIENTATION_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_ORIENTATION_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_PSTABILIZATION_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_PSTABILIZATION_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_NMEA_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_NMEA_CMD_RECEIVE_SIZE;
		break;
	case IL_IMU_GET_DEV_INFO_RECEIVE:
		IMUInt->num_bytes_recive = IL_IMU_GET_DEV_INFO_CMD_RECEIVE_SIZE;
		break;
	case IL_READ_IMU_PAR_RECEIVE:
		IMUInt->num_bytes_recive = IL_READ_IMU_PAR_CMD_RECEIVE_SIZE;
		break;
	case IL_LOAD_IMU_PAR_RECEIVE:
		IMUInt->num_bytes_recive = IL_LOAD_IMU_PAR_CMD_RECEIVE_SIZE;
		break;
	default:
		break;
	}
}

IL_BOOL IMU_shouldCheckForResponse_threadSafe(IL_IMU* imu, char* responseMatchBuffer, int num_bytes_to_recive)
{
	IMUInternal* IMUInt;
	IL_BOOL shouldCheckResponse;

	IMUInt = IMU_getInternalData(imu);

	inertial_criticalSection_enter(&IMUInt->critSecForResponseMatchAccess);
	shouldCheckResponse = 1; // to avoid warning now
	inertial_criticalSection_leave(&IMUInt->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}


void IMU_disableResponseChecking_threadSafe(IL_IMU* imu)
{
	IMUInternal* IMUInt;

	IMUInt = IMU_getInternalData(imu);

	inertial_criticalSection_enter(&IMUInt->critSecForResponseMatchAccess);

	IMUInt->checkForResponse = IL_FALSE;
	IMUInt->cmdResponseSize = 0;
	IMUInt->cmdResponseMatchBuffer[0] = 0;

	inertial_criticalSection_leave(&IMUInt->critSecForResponseMatchAccess);
}


IL_ERROR_CODE IMU_registerDataReceivedListener(IL_IMU* imu, IMUNewDataReceivedListener listener)
{
	IMUInternal* IMUInt = IMU_getInternalData(imu);

	if (IMUInt->DataListener != NULL)
		return ILERR_UNKNOWN_ERROR;

	IMUInt->DataListener = listener;

	return ILERR_NO_ERROR;
}

IL_ERROR_CODE IMU_unregisterDataReceivedListener(IL_IMU* imu, IMUNewDataReceivedListener listener)
{
	IMUInternal* IMUInt = IMU_getInternalData(imu);

	if (IMUInt->DataListener == NULL)
		return ILERR_UNKNOWN_ERROR;

	if (IMUInt->DataListener != listener)
		return ILERR_UNKNOWN_ERROR;

	IMUInt->DataListener = NULL;

	return ILERR_NO_ERROR;
}

unsigned char IMU_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char)cmdToCheck[i];

	return xorVal;
}

void IMU_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = IMU_checksum_compute(cmdToCheck);
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

void IMU_processAsyncData(IL_IMU* imu, unsigned char buffer[])
{
	IMUCompositeData data;
	IMUInternal* IMUInt = IMU_getInternalData(imu);

	memset(&data, 0, sizeof(IMUCompositeData));


	/* We had an async data packet and need to move it to IMUInt->lastestAsyncData. */
	inertial_criticalSection_enter(&IMUInt->critSecForLatestAsyncDataAccess);
	memcpy(&IMUInt->lastestAsyncData, &data, sizeof(IMUCompositeData));
	inertial_criticalSection_leave(&IMUInt->critSecForLatestAsyncDataAccess);

	if (IMUInt->DataListener != NULL)
		IMUInt->DataListener(imu, &IMUInt->lastestAsyncData);


}




/** \endcond */