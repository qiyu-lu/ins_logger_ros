/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to devices based on Inertial Labs,
 * family of orientation sensors.
 */

#ifndef _INERTIALLABS_IMU_H_
#define _INERTIALLABS_IMU_H_

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

#define IL_SET_CONTINUES_MODE     				0    /**< Continues data receive Mode on. */
#define IL_SET_ONREQUEST_MODE 	  				1    /**< OnrequestMode data receive Mode on. */

#define IL_STOP_CMD          					0     /**< Stop command flag. */
#define IL_SET_ONREQUEST_CMD 			     	1	   /**< OnRequestMode command flag. */
#define IL_IMU_CLB_DATA_RECEIVE 	 			2     
#define IL_IMU_GA_DATA_RECEIVE      			3      
#define IL_IMU_ORIENTATION_RECEIVE				4
#define IL_IMU_PSTABILIZATION_RECEIVE			5
#define IL_IMU_NMEA_RECEIVE						6
#define IL_READ_IMU_PAR_RECEIVE					7
#define IL_LOAD_IMU_PAR_RECEIVE     			8      /**< LoadINSpar command flag. */
#define IL_IMU_GET_DEV_INFO_RECEIVE				9


#define IL_STOP_CMD_RECEIVE_SIZE            		0         /**< After Stop command no of bytes received. */
#define IL_SET_ONREQUEST_CMD_RECEIVE_SIZE   		146       /**< After OnRequestMode command no of bytes received. */
#define IL_IMU_CLB_DATA_CMD_RECEIVE_SIZE         	37       /**< After INS OPVT command no of bytes need to receive. */
#define IL_IMU_GA_DATA_CMD_RECEIVE_SIZE            	39       /**< After INS QPVT command no of bytes need to receive. */
#define IL_IMU_ORIENTATION_CMD_RECEIVE_SIZE			41
#define IL_IMU_PSTABILIZATION_CMD_RECEIVE_SIZE		29
#define IL_IMU_NMEA_CMD_RECEIVE_SIZE				60
#define IL_IMU_GET_DEV_INFO_CMD_RECEIVE_SIZE		56
#define IL_READ_IMU_PAR_CMD_RECEIVE_SIZE			68
#define IL_LOAD_IMU_PAR_CMD_RECEIVE_SIZE			2


	/**
	 * \brief Holds connection information for accessing a IMU-P device.
	 */
	typedef struct {
		char*	portName;		/**< The name of the serial port. */
		int		baudRate;		/**< The baudrate of the serial port. */
		IL_BOOL	isConnected;	/**< Inidicates if the serial port is open. */
		void*	internalData;	/**< Pointer to data used internally by the IMU function handlers. */
		int     mode;           /**< Inidicates the mode of communication (OnrequestMode/Continues). */
		int 	cmd_flag;       /**< Inidicates the IMU command Macros in use.*/
	}IL_IMU;

	/**
	 * \brief Composite structure of the various  data the IMU-P device
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
	} IMUCompositeData;

	/**
	 * \brief Read Internal Parameters Data structure of the various  data the IMU-P device
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

		double    Alignment_Angle_A1;
		double    Alignment_Angle_A2;
		double    Alignment_Angle_A3;

		double    IMU_Mount_Right;
		double    IMU_Mount_Forward;
		double    IMU_Mount_Up;

		double    Antenna_Pos_Right;
		double    Antenna_Pos_Forward;
		double    Antenna_Pos_Up;

		int       Altitude_one_byte;

		char    *IMU_Device_Name;
		int       Baro_Altimeter;

	} IMUSetInternalData;


	/**
	 * \brief Fuction type used for receiving notifications of when new asynchronous
	 * data is received.
	 *
	 * \param[in]	sender	The device that sent the notification.
	 * \param[in]	newData	Pointer to the new data.
	 */
	typedef void(*IMUNewDataReceivedListener)(IL_IMU* sender, IMUCompositeData* newData);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the IMU-P module.
	 *
	 * \param[in] imu Pointer to the IMU control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_registerAsyncDataReceivedListener(IL_IMU* imu, IMUNewDataReceivedListener listener);

	/**
	 * \brief Connects to a InertialLab IMU-P device.
	 *
	 * \param[out]	newIMU   	An uninitialized IMU control object should be passed in.
	 * \param[in]	portName	The name of the COM port to connect to.
	 * \param[in]	baudrate	The baudrate to connect at.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE IMU_connect(IL_IMU* newIMU, const char* portName, int baudrate);

	/**
	 * \brief Disconnects from the IMU-P device and disposes of any internal resources.
	 *
	 * \param imu Pointer to the IMU control object.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_disconnect(IL_IMU* imu);

	/**
	 * \brief Checks if we are able to send and receive communication with the InertialLabs IMU-P sensor.
	 *
	 * \param[in] imu Pointer to the IL_IMU control object.
	 *
	 * \return IL_TRUE if the library was able to send and receive a valid response from the InertialLabs IMU sensor; otherwise IL_FALSE.
	 */
	DLL_EXPORT IL_BOOL IMU_verifyConnectivity(IL_IMU* imu);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the module.
	 *
	 * \param[in] imu Pointer to the IMU control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_registerDataReceivedListener(IL_IMU* imu, IMUNewDataReceivedListener listener);

	/**
	 * \brief Unregisters an already registered function for recieving notifications
	 * of when new asynchronous data is received.
	 *
	 * \param[in] imu Pointer to the IMU  control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_unregisterAsyncDataReceivedListener(IL_IMU* imu, IMUNewDataReceivedListener listener);

	/**
	 * \brief Computes the checksum for the provided command.
	 *
	 * \param[in]	cmdToCheck
	 *
	 *
	 * \return The computed checksum number.
	 */
	DLL_EXPORT unsigned char IMU_checksum_compute(const char* cmdToCheck);

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
	DLL_EXPORT void IMU_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);

	/**
	 * \brief Send the stop command to the IMU to stop any communication with the device.
	 *        After  calling this function IMU goes to idle mode and redy to recive any commands .
	 *
	 * \param	imu Pointer to the IMU control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_Stop(IL_IMU* imu);

	/**
	 * \brief Set the data output mode of the IMU . There are two modes available Continues & OnrequestMode
	 *
	 * \param[in]   imu    Pointer to the IMU control object.
	 * \param[in]	mode   IL_CONTINUES_MODE / IL_SET_ONREQUEST_MODE
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_SetMode(IL_IMU* imu, int mode);

	/**
	 * \brief Send the OnRequestMode payload to the IMU.
	 *        If this function  called directly , you must add 30 sec delay after this function.
	 *        IMU calibaration will take 30 secs .
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 * \param[in]   syncDataOutputType Set the data speed
	 * \param[in]   waitForResponse set the flag to process the response .
	 * 			    if 0 , SDK will not process the calibaration output packet from IMU .
	 *              if 1 , SDK will process the  calibaration output packet from IMU .
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_setSetOnRequestMode(IL_IMU* imu, unsigned int syncDataOutputType, IL_BOOL waitForResponse);

	/**
	 * \brief Deliver the payload to recive data in  “IMU ClbData” data format from IMU.
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_ClbData_Receive(IL_IMU* imu);

	/**
	 * \brief Deliver the payload to recive data in  “IMU GAdata” data format from IMU.
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 *
	 * \param[out]   dataBuffer  Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_GAdata_Receive(IL_IMU* imu);

	/**
	 * \brief Deliver the payload to recive data in  “IMU Orientation” data format from IMU.
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_Orientation_Receive(IL_IMU* imu);

	/**
	 * \brief Deliver the payload to recive data in  “IMU PStabilization” data format from IMU.
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_PStabilization_Receive(IL_IMU* imu);


	/**
	 * \brief Deliver the payload to recive data in  “IMU NMEA ” data format from IMU.
	 *
	 * \param[in]	imu         Pointer to the IMU control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_NMEA_Receive(IL_IMU* imu);


	/**
	 * \brief Gets the values of the Heading , Pitch , Roll and Calibrated Measurements .
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 * \param[out]  data The current sensor Yaw Pitch Roll values.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_YPR(IL_IMU* imu, IMUCompositeData* data);

	/**
	 * \brief Gets the values of the Gyro , Acceleration , Magnetic , Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 * \param[out]  Gyro The current sensor Gyro (X,Y,Z) values .
	 * \param[out]	Acceleration The current sensor acceleration (X ,Y,Z) values.
	 * \param[out]  Magnetic The current sensor magnetic (X,Y,Z) values.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_getGyroAccMag(IL_IMU* imu, IMUCompositeData* data);

	/**
	 * \brief Gets the values of the  Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	imu Pointer to the IMU control object.
	 *
	 * \param[out]  VInp  Input voltage to the INS
	 * \param[out]  Temper of Temprature value
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_getSensorData(IL_IMU* imu, IMUCompositeData* data);

	/**
	 * \brief Deliver the payload to recive data in   IMU parameters (60 bytes) from the IMU nonvolatile memory.
	 *
	 * \param[in]	imu   Pointer to the IMU control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE ReadIMUpar(IL_IMU* imu);


	/**
	 * \brief Deliver the IMU parameters (60 bytes) to  IMU nonvolatile memory.
	 *
	 * \param[in]	imu        Pointer to the IMU control object.
	 *
	 *
	 * \param[out]  check sum value in  the output packet , saved in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE LoadIMUpar(IL_IMU* imu);

	/**
	 * \brief 
	 *
	 * \param[in]	imu        Pointer to the IMU control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer  .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE IMU_GetDevInfo(IL_IMU* imu);



	DLL_EXPORT IL_ERROR_CODE IMU_ReadInternalParameters(IL_IMU* imu, IMUSetInternalData* data);



#ifdef __cplusplus
}
#endif

#endif /* _INERTIALLABS_IMU_H__ */
