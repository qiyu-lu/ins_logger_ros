/**
 * \file
 
 *
 * \section DESCRIPTION
 * This header file provides access to the Inertial Labs Kinematics library.
 */
#ifndef _IL_KINEMATICS_H
#define _IL_KINEMATICS_H

/**
 * \brief Holds attitude data expressed in yaw, pitch, roll format.
 */
typedef struct {
	double	yaw;		/**< Yaw */
	double	    pitch;		/**< Pitch */
	double	    roll;		/**< Roll */
} IlYpr;

/**
 * \brief Holds attitude data expressed in quaternion format.
 */
typedef struct {
	double Lk0;		
	double Lk1;		
	double Lk2;		
	double Lk3;		
} IlQuaternion;

#endif /* _IL_KINEMATICS_H */
