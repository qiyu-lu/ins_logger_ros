/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to devices based on Inertial Labs,
 * family of orientation sensors.
 */

#ifndef _INERTIALLABS_AHRS_H_
#define _INERTIALLABS_AHRS_H_

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

#define IL_AHRS_STOP_CMD          			1     /**< Stop command flag. */
#define IL_AHRS_CONT_1_CMD 			     	2
#define IL_AHRS_CONT_2_CMD 			     	3
#define IL_AHRS_CONT_3_CMD 			     	4
#define IL_AHRS_REQ_1_CMD 			     	5
#define IL_AHRS_REQ_2_CMD 			     	6
#define IL_AHRS_REQ_3_CMD 			     	7
#define IL_AHRS_NMEA_CONT_CMD               8
#define IL_AHRS_NMEA_REQ_CMD                9
#define IL_AHRS_GETDATA_REQ_CMD             10
#define IL_READ_AHRS_PAR_RECEIVE			11
#define IL_LOAD_AHRS_PAR_RECEIVE     	    12
#define IL_AHRS_LOWPOWER_ON_CMD             13 
#define IL_AHRS_LOWPOWER_OFF_CMD            14
#define IL_AHRS_GET_BIT_CMD                 15  
#define IL_AHRS_GET_VER_FIRMWARE_CMD        16


//User Defined Data

#define IL_AHRS_STOP_CMD_RECEIVE_SIZE            		   0       
#define IL_AHRS_FULL_OUTPUT_RECEIVE_SIZE   		          39
#define IL_AHRS_ORIENTATION_OUTPUT_RECEIVE_SIZE           39
#define IL_AHRS_ORIENTATION_SENSOR_OUTPUT_RECEIVE_SIZE    39
#define IL_AHRS_NMEA_CMD_RECEIVE_SIZE				      60
#define IL_AHRS_GET_FIRMWARE_VER_RECEIVE_SIZE             50
#define IL_READ_AHRS_PAR_CMD_RECEIVE_SIZE		          55
#define IL_AHRS_GET_BIT_RECEIVE_SIZE		               4
#define IL_LOAD_AHRS_PAR_CMD_RECEIVE_SIZE			       2  
#define IL_AHRS_INITIAL_ALIGNMENT_RECEIVE_SIZE	          55

#define IL_AHRS_DBG 1                   /**< SDK Debug mode off. */
#define IL_AHRS_RAW_DATA 1
#define IL_AHRS_DECODE_DATA 1
	/**
	 * \brief Holds connection information for accessing a AHRS device.
	 */
	typedef struct {
		char*	portName;		/**< The name of the serial port. */
		int		baudRate;		/**< The baudrate of the serial port. */
		IL_BOOL	isConnected;	/**< Inidicates if the serial port is open. */
		void*	internalData;	/**< Pointer to data used internally by the AHRS function handlers. */
		int     mode;           /**< Inidicates the mode of communication (OnrequestMode/Continues). */
		int 	cmd_flag;       /**< Inidicates the AHRS command Macros in use.*/
	}IL_AHRS;

	/**
	 * \brief Composite structure of the various  data the AHRS device
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
	} AHRSCompositeData;

	/**
	 * \brief Read Internal Parameters Data structure of the various  data the AHRS device
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

	} AHRS_SetInternalData;


	/**
	 * \brief Fuction type used for receiving notifications of when new asynchronous
	 * data is received.
	 *
	 * \param[in]	sender	The device that sent the notification.
	 * \param[in]	newData	Pointer to the new data.
	 */
	typedef void(*AHRS_NewDataReceivedListener)(IL_AHRS* sender, AHRSCompositeData* newData);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the AHRS module.
	 *
	 * \param[in] AHRS Pointer to the AHRS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_registerAsyncDataReceivedListener(IL_AHRS* ahrs, AHRS_NewDataReceivedListener listener);

	/**
	 * \brief Connects to a InertialLab AHRS device.
	 *
	 * \param[out]	newAHRS   	An uninitialized AHRS control object should be passed in.
	 * \param[in]	portName	The name of the COM port to connect to.
	 * \param[in]	baudrate	The baudrate to connect at.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE AHRS_connect(IL_AHRS* newAHRS, const char* portName, int baudrate);

	/**
	 * \brief Disconnects from the AHRS device and disposes of any internal resources.
	 *
	 * \param ahrs Pointer to the AHRS control object.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_disconnect(IL_AHRS* ahrs);

	/**
	 * \brief Checks if we are able to send and receive communication with the InertialLabs AHRS sensor.
	 *
	 * \param[in] ahrs Pointer to the IL_AHRS control object.
	 *
	 * \return IL_TRUE if the library was able to send and receive a valid response from the InertialLabs AHRS sensor; otherwise IL_FALSE.
	 */
	DLL_EXPORT IL_BOOL AHRS_verifyConnectivity(IL_AHRS* ahrs);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the module.
	 *
	 * \param[in] ahrs Pointer to the AHRS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_registerDataReceivedListener(IL_AHRS* ahrs, AHRS_NewDataReceivedListener listener);

	/**
	 * \brief Unregisters an already registered function for recieving notifications
	 * of when new asynchronous data is received.
	 *
	 * \param[in] ahrs Pointer to the AHRS  control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_unregisterAsyncDataReceivedListener(IL_AHRS* ahrs, AHRS_NewDataReceivedListener listener);

	/**
	 * \brief Computes the checksum for the provided command.
	 *
	 * \param[in]	cmdToCheck
	 *
	 *
	 * \return The computed checksum number.
	 */
	DLL_EXPORT unsigned char AHRS_checksum_compute(const char* cmdToCheck);

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
	DLL_EXPORT void AHRS_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);

	/**
	 * \brief Send the stop command to the AHRS to stop any communication with the device.
	 *        After  calling this function AHRS goes to idle mode and redy to recive any commands .
	 *
	 * \param	ahrs Pointer to the AHRS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_Stop(IL_AHRS* ahrs);


	/**"
	 * \brief Deliver the payload to recive data in  “AHRScont1" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRScont1_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  “AHRScont2" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRScont2_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  “AHRScont3" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRScont3_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  "AHRSreq1" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRSreq1_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  "AHRSreq2" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRSreq2_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  "AHRSreq3" data format from AHRS.
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE  AHRSreq3_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  “NMEAcont ” data format from AHRS.
	 *
	 * \param[in]	ahrs         Pointer to the AHRS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE NMEAcont_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Deliver the payload to recive data in  NMEAreq ” data format from AHRS.
	 *
	 * \param[in]	ahrs         Pointer to the AHRS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE NMEAreq_Receive(IL_AHRS* ahrs);

	/**
	 * \brief Switches the AHRS to low power “Sleep” mode. 
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_LowPowerOn(IL_AHRS* ahrs);

	/**
	 * \brief Awakes the AHRS from the Sleep mode and switches it to the
     *   idle mode with normal power consumption.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_LowPowerOff(IL_AHRS* ahrs);

    /**
	 * \brief Read firmware version of the AHRS (50 bytes) from the AHRS
     * nonvolatile memory.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_GetVerFirmware(IL_AHRS* ahrs);


    /**
	 * \brief  AHRS has continuous built-in monitoring of its health.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer  .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_GetBIT(IL_AHRS* ahrs);

	/**
	 * \brief Gets the values of the Heading , Pitch , Roll and Calibrated Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 * \param[out]  data The current sensor Yaw Pitch Roll values.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_YPR(IL_AHRS* ahrs, AHRSCompositeData* data);

	/**
	 * \brief Gets the values of the Gyro , Acceleration , Magnetic , Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 * \param[out]  Gyro The current sensor Gyro (X,Y,Z) values .
	 * \param[out]	Acceleration The current sensor acceleration (X ,Y,Z) values.
	 * \param[out]  Magnetic The current sensor magnetic (X,Y,Z) values.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_getGyroAccMag(IL_AHRS* ahrs, AHRSCompositeData* data);

	/**
	 * \brief Gets the values of the  Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	ahrs Pointer to the AHRS control object.
	 *
	 * \param[out]  VInp  Input voltage to the INS
	 * \param[out]  Temper of Temprature value
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_getSensorData(IL_AHRS* ahrs, AHRSCompositeData* data);

	/**
	 * \brief Deliver the payload to recive data in   AHRS parameters (60 bytes) from the AHRS nonvolatile memory.
	 *
	 * \param[in]	ahrs   Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE ReadAHRSpar(IL_AHRS* ahrs);


	/**
	 * \brief Deliver the AHRS parameters (60 bytes) to  AHRS nonvolatile memory.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  check sum value in  the output packet , saved in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE LoadAHRSpar(IL_AHRS* ahrs);

   /**
	 * \brief Deliver the AHRS parameters (60 bytes) to  AHRS nonvolatile memory.
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  check sum value in  the output packet , saved in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE LoadAHRSpar(IL_AHRS* ahrs);

	/**
	 * \brief 
	 *
	 * \param[in]	ahrs        Pointer to the AHRS control object.
	 *
	 *
	 * \param[out]  
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE AHRS_GetDataReq(IL_AHRS* ahrs);
	
	


#ifdef __cplusplus
}
#endif

#endif /* _INERTIALLABS_AHRS_H__ */
