/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to devices based on Inertial Labs,
 * family of orientation sensors.
 */

#ifndef _INERTIALLABS_INS_H_
#define _INERTIALLABS_INS_H_

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
#include "IL_common.h"

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

#define IL_SET_CONTINUES_MODE     		0    /**< Continues data receive Mode on. */
#define IL_SET_ONREQUEST_MODE 	  		1    /**< OnrequestMode data receive Mode on. */

#define IL_STOP_CMD          			0     /**< Stop command flag. */
#define IL_SET_ONREQUEST_CMD 			1     /**< OnRequestMode command flag. */
#define IL_OPVT_RECEIVE 	 			2      /**< INS OPVT data format command flag. */
#define IL_QPVT_RECEIVE      			3      /**< INS QPVT data format command flag. */
#define IL_OPVT2A_RECEIVE    			4      /**< INS OPVT2A data format command flag. */
#define IL_OPVT2AW_RECEIVE   			5      /**< INS OPVT2AW data format command flag. */
#define IL_OPVT2AHR_RECEIVE  			6      /**< INS OPVT2AHR data format command flag. */
#define IL_OPVTAD_RECEIVE    			7      /**< INS OPVTAD data format command flag. */
#define IL_MINIMAL_DATA_RECEIVE 		8      /**< INS Minimal Data format command flag. */
#define IL_SENSOR_DATA_RECEIVE      	9      /**< INS SensorsData data format command flag. */
#define IL_READ_INS_PAR_RECEIVE     	10     /**< ReadINSpar command flag. */
#define IL_OPVT_RAWIMU_DATA_RECEIVE 	11	   /**< INS OPVT & Raw IMU data format command flag. */
#define IL_OPVT_GNSSEXT_DATA_RECEIVE 	12    /**< INS OPVT GNSSext data format command flag. */
#define IL_SPAN_RAWIMU_RECEIVE			13     /**< SPAN rawimu data format command flag. */
#define IL_NMEA_RECEIVE					14     /**< INS NMEA Output data format command flag. */
#define IL_NMEA_SENSORS_RECEIVE			15     /**< INS and Sensors NMEA Output data format command flag. */
#define IL_USERDEF_DATA_RECEIVE			16     /**< User Defined Data format command flag. */
#define IL_DEV_SELF_TEST_RECEIVE    	17      /**< DevSelfTest command flag. */
#define IL_LOAD_INS_PAR_RECEIVE     	18     /**< LoadINSpar command flag. */
#define IL_INITIAL_ALIGNMENT			19		 /**< Initial alignment command flag. */

#define IL_STOP_CMD_RECEIVE_SIZE            	0         /**< After Stop command no of bytes received. */
#define IL_SET_ONREQUEST_CMD_RECEIVE_SIZE   	146       /**< After OnRequestMode command no of bytes received. */
#define IL_OPVT_CMD_RECEIVE_SIZE         		100       /**< After INS OPVT command no of bytes need to receive. */
#define IL_QPVT_CMD_RECEIVE_SIZE            	102       /**< After INS QPVT command no of bytes need to receive. */
#define IL_OPVT2A_CMD_RECEIVE_SIZE          	109		  /**< After INS OPVT2A command no of bytes need to receive. */
#define IL_OPVT2AW_CMD_RECEIVE_SIZE         	111       /**< After INS OPVT2AW command no of bytes need to receive. */
#define IL_OPVT2AHR_CMD_RECEIVE_SIZE        	137       /**< After INS OPVT2AHR command no of bytes need to receive. */
#define IL_OPVTAD_CMD_RECEIVE_SIZE          	185       /**< After INS OPVTAD command no of bytes need  to receive. */
#define IL_MINIMAL_DATA_CMD_RECEIVE_SIZE    	50        /**< After INS OPVT command no of bytes need to receive. */
#define IL_DEV_SELF_TEST_CMD_RECEIVE_SIZE   	1         /**< After DevSelfTest command no of bytes need to receive. */
#define IL_READ_INS_PAR_CMD_RECEIVE_SIZE    	68        /**< After ReadINSpar command no of bytes need to receive. */
#define IL_LOAD_INS_PAR_CMD_RECEIVE_SIZE    	2         /**< After LoadINSpar command no of bytes need to receive. */
#define IL_SENSOR_DATA_CMD_RECEIVE_SIZE     	92        /**< After INS SensorsData command no of bytes need to receive. */
#define IL_NMEA_CMD_RECEIVE_SIZE				00		  /**< After INS NMEA command no of bytes need to receive. */
#define IL_NMEA_SENSORS_CMD_RECEIVE_SIZE		00        /**< After INS NMEA Sensors  command no of bytes need to receive. */
#define IL_OPVT_RAWIMU_DATA_CMD_RECEIVE_SIZE    98        /**< After INS OPVT & Raw IMU data command no of bytes need to receive. */
#define IL_OPVT_GNSSEXT_DATA_CMD_RECEIVE_SIZE	194       /**< After INS OPVT GNSSext data command no of bytes need to receive. */
#define IL_USERDEF_DATA_CMD_RECEIVE_SIZE        00        /**< After User Defined Data command no of bytes need to receive. */
#define IL_SPAN_RAWIMU_CMD_RECEIVE_SIZE         72        /**< After SPAN rawimu command no of bytes need to receive. */
#define IL_INITIAL_ALIGNMENT_SHORT_SIZE			58		  /**< Initial alignment short bytes need to receive. */	
#define IL_INITIAL_ALIGNMENT_EXTENDED_SIZE 		136		  /**< initial alignment extended bytes need to receive. */	
	/**
	 * \brief Holds connection information for accessing a INS device.
	 */
	typedef struct {
		char* portName;		/**< The name of the serial port. */
		int		baudRate;		/**< The baudrate of the serial port. */
		IL_BOOL	isConnected;	/**< Inidicates if the serial port is open. */
		void* internalData;	/**< Pointer to data used internally by the INS function handlers. */
		int     mode;           /**< Inidicates the mode of communication (OnrequestMode/Continues). */
		int 	cmd_flag;       /**< Inidicates the INS command Macros in use.*/
	}IL_INS;

	/**
	 * \brief Composite structure of the various  data the INS device
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
		double 			H_bar;						/**< Barometric height measurements . */
		double 			P_bar;                      /**< Pressure measurements . */
	} INSCompositeData;

	/**
	 * \brief Position Data structure of the various  data the INS device
	 * is capable of outputting.
	 */

	typedef struct {

		double    Latitude;
		double 	  Longitude;
		double 	  Altitude;
		double 	  East_Speed;
		double 	  North_Speed;
		double 	  Vertical_Speed;
		double 	  GNSS_Latitude;
		double    GNSS_Longitude;
		double    GNSS_Altitude;
		double    GNSS_Horizontal_Speed;
		double    GNSS_Trackover_Ground;
		double    GNSS_Vertical_Speed;


		double    GNSS_Heading;
		double 	  GNSS_Pitch;
		double    GNSS_Heading_STD;
		double    GNSS_Pitch_STD;

		double    Latency_ms_head;
		double    Latency_ms_pos;
		double 	  Latency_ms_vel;

		double    V_Latency;

	} INSPositionData;




	/**
	 * \brief Read Internal Parameters Data structure of the various  data the INS device
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

		double    INS_Mount_Right;
		double    INS_Mount_Forward;
		double    INS_Mount_Up;

		double    Antenna_Pos_Right;
		double    Antenna_Pos_Forward;
		double    Antenna_Pos_Up;

		int       Altitude_one_byte;

		char* INS_Device_Name;
		int       Baro_Altimeter;

	} INSSetInternalData;



	typedef struct {

		float	  Gyros_bias1;
		float	  Gyros_bias2;
		float	  Gyros_bias3;
		float    Average_acceleration1;
		float    Average_acceleration2;
		float    Average_acceleration3;
		float    Average_magnfield1;
		float    Average_magnfield2;
		float    Average_magnfield3;
		float    Initial_Heading;
		float 	 Initial_Roll;
		float    Initial_Pitch;

		int       USW;

		float    temp_gyro1;
		float    temp_gyro2;
		float    temp_gyro3;

		float    temp_acc1;
		float    temp_acc2;
		float    temp_acc3;

		float    temp_mag1;
		float    temp_mag2;
		float    temp_mag3;


		double	  temp_pressure_sensor;
		double	  pressure_data;

		double 	  Latitude;
		double    Longitude;
		double 	  Altitude;

		float     Velocity_V_east;
		float 	  Velocity_V_north;
		float     Velocity_V_up;
		double    Gravity_G;



	} INSSetInitialData;


	/**
	 * \brief Read SPAN rawimu command Data structure of the various  data the INS device
	 * is capable of outputting.
	 */

	typedef struct {

		int 		port_address;
		int 		message_length;
		int 		sequence;
		int 		idle_time;
		int 		time_status;
		double 		week;
		double 		gps_time;
		double 		receiver_status;
		double 		receiver_sw_version;
		double 		week_4byte;
		double 		gps_imu_time;
		double 		imu_status;

		IlVector3	acceleration;
		IlVector3	gyro;



	} INSRawImuData;

	/**
	 * \brief Read GNSS solution status & GNSS position type  details from INS sensor data command .
	 */

	typedef struct {

		int 		sol_stat;
		int 		pos_type;

	} INSGnssData;


	/**
	 * \brief Fuction type used for receiving notifications of when new asynchronous
	 * data is received.
	 *
	 * \param[in]	sender	The device that sent the notification.
	 * \param[in]	newData	Pointer to the new data.
	 */
	typedef void(*INSNewDataReceivedListener)(IL_INS* sender, INSCompositeData* newData);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the INS module.
	 *
	 * \param[in] ins Pointer to the INS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_registerAsyncDataReceivedListener(IL_INS* ins, INSNewDataReceivedListener listener);

	/**
	 * \brief Connects to a InertialLab INS device.
	 *
	 * \param[out]	newINS   	An uninitialized INS control object should be passed in.
	 * \param[in]	portName	The name of the COM port to connect to.
	 * \param[in]	baudrate	The baudrate to connect at.
	 * \return InertialLab error code.
	 */
	IL_ERROR_CODE INS_connect(IL_INS* newINS, const char* portName, int baudrate);

	/**
	 * \brief Disconnects from the INS device and disposes of any internal resources.
	 *
	 * \param INS Pointer to the INS control object.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_disconnect(IL_INS* ins);

	/**
	 * \brief Checks if we are able to send and receive communication with the InertialLabs Ins sensor.
	 *
	 * \param[in] ins Pointer to the IL_INS control object.
	 *
	 * \return IL_TRUE if the library was able to send and receive a valid response from the InertialLabs Ins sensor; otherwise IL_FALSE.
	 */
	DLL_EXPORT IL_BOOL INS_verifyConnectivity(IL_INS* ins);

	/**
	 * \brief Allows registering a function which will be called whenever a new
	 * asynchronous data packet is received from the module.
	 *
	 * \param[in] ins Pointer to the INS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_registerDataReceivedListener(IL_INS* ins, INSNewDataReceivedListener listener);

	/**
	 * \brief Unregisters an already registered function for recieving notifications
	 * of when new asynchronous data is received.
	 *
	 * \param[in] ins Pointer to the INS  control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_unregisterAsyncDataReceivedListener(IL_INS* ins, INSNewDataReceivedListener listener);

	/**
	 * \brief Computes the checksum for the provided command.
	 *
	 * \param[in]	cmdToCheck
	 *
	 *
	 * \return The computed checksum number.
	 */
	DLL_EXPORT unsigned char INS_checksum_compute(const char* cmdToCheck);

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
	DLL_EXPORT void INS_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);

	/**
	 * \brief Send the stop command to the INS to stop any communication with the device.
	 *        After  calling this function INS goes to idle mode and redy to recive any commands .
	 *
	 * \param	ins Pointer to the INS control object.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_Stop(IL_INS* ins);

	/**
	 * \brief Set the data output mode of the INS . There are two modes available Continues & OnrequestMode
	 *
	 * \param[in]   ins    Pointer to the INS control object.
	 * \param[in]	mode   IL_CONTINUES_MODE / IL_SET_ONREQUEST_MODE
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_SetMode(IL_INS* ins, int mode);

	/**
	 * \brief Send the OnRequestMode payload to the INS .
	 *        If this function  called directly , you must add 30 sec delay after this function.
	 *        INS calibaration will take 30 secs .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 * \param[in]   syncDataOutputType Set the data speed
	 * \param[in]   waitForResponse set the flag to process the response .
	 * 			    if 0 , SDK will not process the calibaration output packet from INS .
	 *              if 1 , SDK will process the  calibaration output packet from INS .
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_setSetOnRequestMode(IL_INS* ins, unsigned int syncDataOutputType, IL_BOOL waitForResponse);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT” data format from INS.
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVTdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS QPVT” data format from INS.
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 *
	 * \param[out]   dataBuffer  Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_QPVTdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT2A” data format from INS.
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVT2Adata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT2AW” data format from INS.
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVT2AWdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT2AHR” data format from INS.
	 *
	 * \param[in]	ins       Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVT2AHRdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVTAD” data format from INS.
	 *
	 * \param[in]	ins        Pointer to the INS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVTADdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS Minimal Data” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_Minimaldata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS SensorsData Data” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_SensorsData_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS NMEA ” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_NMEA_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS Sensors NMEA” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_Sensors_NMEA_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “User Defined Data” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE UserDef_Data_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “SPAN rawimu” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE SPAN_rawimu_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT GNSSext” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVT_GNSSextdata_Receive(IL_INS* ins);

	/**
	 * \brief Deliver the payload to recive data in  “INS OPVT & Raw IMU” data format from INS.
	 *
	 * \param[in]	ins         Pointer to the INS control object.
	 *
	 *
	 * \param[out] dataBuffer    Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_OPVT_rawIMUdata_Receive(IL_INS* ins);

	/**
	 * \brief Gets the values of the Heading , Pitch , Roll and Calibrated Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 * \param[out]  data The current sensor Yaw Pitch Roll values.
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_YPR(IL_INS* ins, INSCompositeData* data);

	/**
	 * \brief Gets the values of the Gyro , Acceleration , Magnetic , Input Voltage and Temprature Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  Gyro The current sensor Gyro (X,Y,Z) values .
	 * \param[out]	Acceleration The current sensor acceleration (X ,Y,Z) values.
	 * \param[out]  Magnetic The current sensor magnetic (X,Y,Z) values.
	 * \param[out]  VInp  Input voltage to the INS
	 * \param[out]  Temper of Temprature value
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getGyroAccMag(IL_INS* ins, INSCompositeData* data);

	/**
	 * \brief Gets the values of the Longitude , Latitude , Altitude , East Speed, North Speed,
	 *        Vertical speed , Latitude GNSS , Longitude GNSS ,Altitude GNSS ,  Horizontal Speed GNSS,
	 *        Trackover Ground GNSS and Vertical Speed GNSS  Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  Longitude Position value.
	 * \param[out]	Latitude Position value.
	 * \param[out]  Altitude Position value.
	 * \param[out]  East_Speed velocity value
	 * \param[out]  North_Speed velocity value
	 * \param[out]  Vertical_Speed velocity value
	 * \param[out]  GNSS_Data Latitude,Longitude,Altitude values
	 * \param[out]  GNSS_Velocity  Horizontal,Trackover,Vertical velocity values
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getPositionData(IL_INS* ins, INSPositionData* data);

	/**
	 * \brief Gets the values of the Heading GNSS  , Pitch GNSS, Heading STD GNSS and Pitch STD GNSS Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  GNSS_Heading
	 * \param[out]	GNSS_Pitch
	 * \param[out]  GNSS_Heading_STD
	 * \param[out]  GNSS_Pitch_STD

	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getGNSS_HeadPitch(IL_INS* ins, INSPositionData* data);

	/**
	 * \brief Gets the values of the Latency ms_head, Latency ms_pos, and Latency ms_vel Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  Latency_ms_head
	 * \param[out]	Latency_ms_pos
	 * \param[out]  GNSS_Heading_STD
	 * \param[out]  Latency_ms_vel
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getLatencyData(IL_INS* ins, INSPositionData* data);

	/**
	 * \brief Gets the values of  pressure and barometric height  Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  H_bar   Barometric height measurements.
	 * \param[out]	P_bar   Pressure measurements.

	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getPressureBarometricData(IL_INS* ins, INSCompositeData* data);

	/**
	 * \brief Gets the values of  GNSS solution status & GNSS position type  details from INS sensor data command .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  sol_stat   GNSS solution status.
	 * \param[out]	pos_type   GNSS position type.

	 * \return InertialLab error code.
	 */

	DLL_EXPORT IL_ERROR_CODE INS_GNSS_Details(IL_INS* ins, INSGnssData* data);

	/**
	 * \brief Gets the  Quaternion value of orientation  Measurements .
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  Quaternion   The current sensor orientation (Lk0,Lk1,Lk2,Lk3) values.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_getQuaternionData(IL_INS* ins, INSCompositeData* data);


	/**
	 * \cond INCLUDE_PRIVATE
	 * \brief Gets result of the device self-test during INS initialization after power on.
	 *
	 * \param[in]	ins Pointer to the INS control object.
	 *
	 * \param[out]  Quaternion   The current sensor orientation (Lk0,Lk1,Lk2,Lk3) values.
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_DevSelfTest(IL_INS* ins);


	/**
	 * \brief Deliver the payload to recive data in   INS parameters (60 bytes) from the INS nonvolatile memory.
	 *
	 * \param[in]	ins        Pointer to the INS control object.
	 *
	 *
	 * \param[out]  dataBuffer Save the output packet in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_ReadINSpar(IL_INS* ins);


	/**
	 * \brief Deliver the INS parameters (60 bytes) to  INS nonvolatile memory.
	 *
	 * \param[in]	ins        Pointer to the INS control object.
	 *
	 *
	 * \param[out]  check sum value in  the output packet , saved in the dataBuffer .
	 *
	 * \return InertialLab error code.
	 */
	DLL_EXPORT IL_ERROR_CODE INS_LoadINSpar(IL_INS* ins);


	DLL_EXPORT IL_ERROR_CODE INS_Alignment(IL_INS* ins, INSSetInitialData* data);




	DLL_EXPORT IL_ERROR_CODE INS_ReadInternalParameters(IL_INS* ins, INSSetInternalData* data);

	DLL_EXPORT IL_ERROR_CODE INS_ReadRawImuParameters(IL_INS* ins, INSRawImuData* data);


#ifdef __cplusplus
}
#endif

#endif /* _INERTIALLABS_INS_H__ */
