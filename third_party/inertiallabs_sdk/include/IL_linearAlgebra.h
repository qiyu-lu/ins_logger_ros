/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides access to the linear algebra features.
 */
#ifndef _IL_LINEAR_ALGEBRA_H_
#define _IL_LINEAR_ALGEBRA_H_

/**
 * \brief A vector of length 3.
 */
typedef struct {
	double	c0;		
	double	c1;		
	double	c2;		
} IlVector3;

/**
 * \brief A 3x3 matrix.
 */
typedef struct {
	double c00;		
	double c01;		
	double c02;		
	double c10;		
	double c11;		
	double c12;		
	double c20;		
	double c21;		
	double c22;	
} IlMatrix3x3;

#endif /* _IL_LINEAR_ALGEBRA_H_ */