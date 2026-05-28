/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to devices based on Inertial Labs,
 * family of orientation sensors.
 */

#ifndef _INERTIALLABS_AHRS_10_H_
#define _INERTIALLABS_AHRS_10_H_

 /* Disable some unnecessary warnings for compiling using Visual Studio with -Wall. */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4668)	/* Preprocessor macro not defined warning. */
#endif

#if defined(_MSC_VER) && _MSC_VER <= 1500
	/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
#include "IL_int.h"
#else
#include <stdint.h>
#endif

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "InertialLabs_services.h"
#include "IL_kinematics.h"
#include "IL_linearAlgebra.h"

#if defined(EXPORT_TO_DLL)
#define DLL_EXPORT __declspec(dllexport)
#else
	/** Don't compile the library with support for DLL function export. */
#define DLL_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ILASYNC_OFF	0		/**< Asynchronous output is turned off. */

/**< AHRS-10 control */

#define IL_AHRS_10_SET_CONTINUES_MODE     				0    /**< Continues data receive Mode on. */
#define IL_AHRS_10_SET_ONREQUEST_MODE 	  				1    /**< OnrequestMode data receive Mode on. */
#define IL_AHRS_10_SET_IDLE_MODE                        2
#define IL_AHRS_10_SET_CALIBRATION_MODE                 3

#define IL_AHRS_10_STOP_CMD          				0     /**< Stop command flag. */
#define IL_AHRS_10_ONREQUEST_CMD 			     	1	   /**< OnRequestMode command flag. */
#define IL_AHRS_10_CLB_DATA_RECEIVE 	 			2
#define IL_AHRS_10_CLB_HR_DATA_RECEIVE 	 			3
#define IL_AHRS_10_NMEA_RECEIVE						4
#define IL_AHRS_10_QUATERNION_RECEIVE				5
#define IL_READ_AHRS_10_PAR_RECEIVE					7
#define IL_LOAD_AHRS_10_PAR_RECEIVE     			8      /**< LoadINSpar command flag. */
#define IL_AHRS_10_GET_DEV_INFO_RECEIVE				9
#define IL_AHRS_10_USER_DEF_DATA_RECEIVE            10



/**< User Defined Data */

#define IL_AHRS_10_STOP_CMD_RECEIVE_SIZE            	0         /**< After Stop command no of bytes received. */
#define IL_AHRS_10_SET_ONREQUEST_CMD_RECEIVE_SIZE   	146       /**< After OnRequestMode command no of bytes received. */
#define IL_AHRS_10_CLB_DATA_CMD_RECEIVE_SIZE         	39       /**< After  AHRS_ClbData command no of bytes need to receive. */
#define IL_AHRS_10_CLB_DATA_HR_DATA_CMD_RECEIVE_SIZE    60      /**< After AHRS_ClbHRData command no of bytes need to receive. */
#define IL_AHRS_10_QUATERNION_CMD_RECEIVE_SIZE			39
#define IL_AHRS_10_NMEA_CMD_RECEIVE_SIZE				100
#define IL_AHRS_10_GET_DEV_INFO_CMD_RECEIVE_SIZE		169
#define IL_READ_AHRS_10_PAR_CMD_RECEIVE_SIZE			56
#define IL_LOAD_AHRS_10_PAR_CMD_RECEIVE_SIZE			2

#define IL_AHRS_10_DBG 0                   /**< SDK Debug mode off. */
#define IL_AHRS_10_RAW_DATA 1
#define IL_AHRS_10_DECODE_DATA 1
	/**
	 * \brief Holds connection information for accessing a AHRS_10 device.
	 */
	typedef struct {
		char*	portName;		/**< The name of the serial port. */
		int		baudRate;		/**< The baudrate of the serial port. */
		IL_BOOL	isConnected;	/**< Inidicates if the serial port is open. */
		void*	internalData;	/**< Pointer to data used internally by the AHRS_10 function handlers. */
		int     mode;           /**< Inidicates the mode of communication (OnrequestMode/Continues). */
		int 	cmd_flag;       /**< Inidicates the AHRS_10 command Macros in use.*/
	}IL_AHRS_10;

	/**
	 * \brief Composite structure of the various  data the AHRS_10 device
	 * is capable of outputting.
	 */
	typedef struct {
		IlYpr           ypr;                        /**< Heading , Pitch , Roll measurments */
		IlQuaternion    quaternion;                 /**< Quaternion of orientation measurements. */
		IlVector3		gyro;                       /**< Gyro measurements. */
		IlVector3		magnetic;					/**< Magnetic measurements. */
		IlVector3		acceleration;				/**< Acceleration measurements. */
		double          Temper;                     /**< Temprature measurements. */
		double          Vinp;                       /**< Input Voltage measurements. */
	} AHRS_10_CompositeData;

	/**
	 * \brief Read Internal Parameters Data structure of the various  data the AHRS_10 device
	 * is capable of outputting.
	 */

	typedef struct {

		double	  Data_rate;
		double    Initial_Alignment_Time;
		double    Magnetic_Declination;
		double    Latitude;
		double 	  Longitude;
		double    Altitude;

		int       Year;
		int       Month;
		int       Day;
        double    Date;

		double    Alignment_Angle_A1;
		double    Alignment_Angle_A2;
		double    Alignment_Angle_A3;

		char*     Device_Id;

	} AHRS_10_SetInternalData;


	
	/**
	 * \brief GetDevInfo command is used to get detailed
	 *	information about devices installed in the AHRS-10
	 */

	typedef struct {

		char*  ID_sn;
		char*  ID_fw;
		int8_t Press_Sens;
		int8_t IMU_type;
		char*  IMU_sn;
		char*  IMU_fw;
		char*  GNSS_model;
		char*  GNSS_sn;
		char*  GNSS_hw;
		char*  GNSS_fw;

		uint16_t GPS_week;
		int8_t GNSS_data_rate;

	} AHRS_10_GetDevInfoData;


	/**
	 * \brief Fuction type used for receiving notifications of when new asynchronous
	 * data is received.
	 *
	 * \param[in]	sender	The device that sent the notification.
	 * \param[in]	newData	Pointer to the new data.
	 */
	typedef void(*AHRS_10_NewDataReceivedListener)(IL_AHRS_10* sender, AHRS_10_CompositeData* newData);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 *  data packet is received from the AHRS_10 module after read parameter command.
	 *
	 * \param[in] AHRS_10 Pointer to the AHRS_10 control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_ReadInternalParameters(IL_AHRS_10* ahrs, AHRS_10_SetInternalData* data);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the AHRS_10 module.
	 *
	 * \param[in] AHRS_10 Pointer to the AHRS_10 control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_registerAsyncDataReceivedListener(IL_AHRS_10* ahrs, AHRS_10_NewDataReceivedListener listener);


	/**
	 * \brief Connects to a InertialLab AHRS_10 device.
	 *
	 * \param[out]	newAHRS_10   	An uninitialized AHRS_10 control object should be passed in.
	 * \param[in]	portName	The name of the COM port to connect to.
	 * \param[in]	baudrate	The baudrate to connect at.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE AHRS_10_connect(IL_AHRS_10* newAHRS_10, const char* portName, int baudrate);

	/**
	 * \brief Disconnects from the AHRS_10 device and disposes of any internal resources.
	 *
	 * \param ahrs Pointer to the AHRS_10 control object.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_disconnect(IL_AHRS_10* ahrs);

	/**
	 * \brief Checks if we are able to send and receive communication with the InertialLabs AHRS_10 sensor.
	 *
	 * \param[in] ahrs Pointer to the IL_AHRS_10 control object.
	 *
	 * \return IL_TRUE if the library was able to send and receive a valid response from the InertialLabs AHRS_10 sensor; otherwise IL_FALSE.
	 */
	DLL_EXPORT IL_BOOL AHRS_10_verifyConnectivity(IL_AHRS_10* ahrs);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the module.
	 *
	 * \param[in] ahrs Pointer to the AHRS_10 control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_registerDataReceivedListener(IL_AHRS_10* ahrs, AHRS_10_NewDataReceivedListener listener);

	/**
	 * \brief Unregisters an already registered function for recieving notifications
	 * of when new asynchronous data is received.
	 *
	 * \param[in] ahrs Pointer to the AHRS_10  control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_unregisterAsyncDataReceivedListener(IL_AHRS_10* ahrs, AHRS_10_NewDataReceivedListener listener);

	/**
	 * \brief Computes the checksum for the provided command.
	 *
	 * \param[in]	cmdToCheck
	 *
	 *
	 * \return The computed checksum number.
	 */
	DLL_EXPORT unsigned char AHRS_10_checksum_compute(const char* cmdToCheck);

	/**
	 * \brief Computes the checksum for the provided command and returns it as a
	 * two character string representing it in hexidecimal.
	 *
	 * \param[in]	cmdToCheck
	 *
	 *
	 * \param[out]	checksum
	 * A character array of length 2 which the computed checksum will be placed.
	 */
	DLL_EXPORT void AHRS_10_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);

	/**
	 * \brief Send the stop command to the AHRS_10 to stop any communication with the device.
	 *        After  calling this function AHRS_10 goes to idle mode and redy to recive any commands .
	 *
	 * \param	ahrs Pointer to the AHRS_10 control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_Stop(IL_AHRS_10* ahrs);

	/**
	 * \brief Set the data output mode of the AHRS_10 . There are two modes available Continues & OnrequestMode
	 *
	 * \param[in]   ahrs    Pointer to the AHRS_10 control object.
	 * \param[in]	mode   IL_CONTINUES_MODE / IL_SET_ONREQUEST_MODE
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_SetMode(IL_AHRS_10* ahrs, int mode);

	/**
	 * \brief Send the OnRequestMode payload to the AHRS_10.
	 *        If this function  called directly , you must add 30 sec delay after this function.
	 *        AHRS_10 calibaration will take 30 secs .
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 * \param[in]   syncDataOutputType Set the data speed
	 * \param[in]   waitForResponse set the flag to process the response .
	 * 			    if 0 , SDK will not process the calibaration output packet from AHRS_10 .
	 *              if 1 , SDK will process the  calibaration output packet from AHRS_10 .
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_SetOnRequestMode(IL_AHRS_10* ahrs, unsigned int syncDataOutputType, IL_BOOL waitForResponse);

	/**
	 * \brief Deliver the payload to recive data in  “AHRS_10 ClbData” data format from AHRS_10.
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_ClbData_Receive(IL_AHRS_10* ahrs);


	/**
	 * \brief Deliver the payload to recive data in  “AHRS_10 ClbData” data format from AHRS_10.
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_ClbHRData_Receive(IL_AHRS_10* ahrs);


	/**
	 * \brief Deliver the payload to recive data in  “AHRS_10 QuatData data format from AHRS_10.
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_QuatData_Receive(IL_AHRS_10* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  “AHRS_10 NMEA ” data format from AHRS_10.
	 *
	 * \param[in]	ahrs         Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_NMEA_Receive(IL_AHRS_10* ahrs);


	/**
	 * \brief Gets the values of the Heading , Pitch , Roll and Calibrated Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 * \param[out]  data The current sensor Yaw Pitch Roll values.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_YPR(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data);

	/**
	 * \brief Gets the values of the Gyro , Acceleration , Magnetic , Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 * \param[out]  Gyro The current sensor Gyro (X,Y,Z) values .
	 * \param[out]	Acceleration The current sensor acceleration (X ,Y,Z) values.
	 * \param[out]  Magnetic The current sensor magnetic (X,Y,Z) values.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_getGyroAccMag(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data);

	/**
	 * \brief Gets the values of the  Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 * \param[out]  VInp  Input voltage to the INS
	 * \param[out]  Temper of Temprature value
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_getSensorData(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data);

	/**
	 * \brief Gets the values of the  Input Quaternion Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS_10 control object.
	 *
	 * \param[out]  
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_getQuaternionData(IL_AHRS_10* ahrs, AHRS_10_CompositeData* data);



	/**
	 * \brief Deliver the payload to recive data in   AHRS_10 parameters (60 bytes) from the AHRS_10 nonvolatile memory.
	 *
	 * \param[in]	ahrs   Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE ReadAHRS10par(IL_AHRS_10* ahrs);


	/**
	 * \brief Deliver the AHRS_10 parameters (60 bytes) to  AHRS_10 nonvolatile memory.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  check sum value in  the output packet , saved in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE LoadAHRS10par(IL_AHRS_10* ahrs);

	/**
	 * \brief 
	 *
	 * \param[in]	ahrs        Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer  .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_GetDevInfo(IL_AHRS_10* ahrs );


	/**
	 * \brief 
	 *
	 * \param[in]	ahrs        Pointer to the AHRS_10 control object.
	 *
	 *
	 * \param[out]  data Save the output packet in the AHRS_10_GetDevInfoData struct  .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_10_GetDevInfoParameters(IL_AHRS_10* ahrs , AHRS_10_GetDevInfoData* data);


#ifdef __cplusplus
}
#endif

#endif /* _INERTIALLABS_AHRS_10_H__ */
