/**
 * \file
 *
 * \section DESCRIPTION
 * This header file provides definitions for standard integer types since
 * Visual Studio 2008 and earlier do not include the stdint.h header file.
 */
#ifndef _IL_INT_H_
#define _IL_INT_H_

typedef signed __int8		int8_t;
typedef signed __int16		int16_t;
typedef signed __int32		int32_t;
typedef signed __int64		int64_t;
typedef unsigned __int8		uint8_t;
typedef unsigned __int16	uint16_t;
typedef unsigned __int32	uint32_t;
typedef unsigned __int64	uint64_t;

#endif /* _IL_INT_H_ */
