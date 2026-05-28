/**
 * @file
 *
 * @section DESCRIPTION
 * This header file defines the error codes used within the Intertial Labs C/C++
 * Library.
 */
#ifndef _IL_ERRORCODES_H_
#define _IL_ERRORCODES_H_

#if defined(_MSC_VER) && _MSC_VER <= 1500
	/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
	#include "IL_int.h"
#else
	#include <stdint.h>
#endif

typedef int IL_ERROR_CODE;

#define ILERR_NO_ERROR				0
#define ILERR_UNKNOWN_ERROR			1
#define ILERR_NOT_IMPLEMENTED		2
#define ILERR_TIMEOUT				3
#define ILERR_INVALID_VALUE			4
#define ILERR_FILE_NOT_FOUND		5
#define ILERR_NOT_CONNECTED			6
#define ILERR_NO_MEMORY_ERROR       7
#define ILERR_MEMORY_ERROR          8
#define ILERR_RECIVE_SIZE_ERROR     9 
#define ILERR_PERMISSION_DENIED     10 
#define ILERR_DATA_IN_BUFFER 		11

#endif /* _IL_ERRORCODES_H_ */
