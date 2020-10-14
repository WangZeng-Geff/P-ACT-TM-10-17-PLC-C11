/*
 * Copyright: (c) 2006-2007, 2008 Triductor Technology, Inc.
 * All Rights Reserved.
 *
 * File:        types.h
 * Purpose:     The primitive types for the whole project.
 * History:     10/15/2006, created by andyzhou
 */

#ifndef _TYPES_H
#define _TYPES_H

#include <stdio.h>

typedef enum {Disable = 0x0, Enable = 0x1} TYPE_FUNCEN;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FuncState;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, PinStatus;

/* unsigned */
typedef unsigned char               u8;
typedef unsigned short              u16;
typedef unsigned int                u32;
typedef unsigned long int           ulong;
typedef unsigned long long int      u64;

typedef unsigned char               U8;
typedef unsigned short              U16;
typedef unsigned int                U32;
typedef unsigned long int           Ulong;
typedef unsigned long long int      U64;

typedef unsigned char               uint8_t;
typedef unsigned short              uint16_t;
typedef unsigned int                uint32_t;
typedef unsigned long int           ulong_t;
typedef unsigned long long int      uint64_t;

typedef unsigned char               UCHAR;
typedef unsigned short              USHORT;
typedef unsigned int                UINT;
typedef unsigned long int           ULONG;
typedef unsigned long long int      ULLONG;

typedef unsigned char               INT8U;
typedef unsigned short              INT16U;
typedef unsigned int                INT32U;
typedef unsigned long long          INT64U;

typedef volatile unsigned char              vu8;
typedef volatile unsigned short             vu16;
typedef volatile unsigned int               vu32;
typedef volatile unsigned long int          vulong;
typedef volatile unsigned long long int     vu64;

typedef volatile unsigned char              VU8;
typedef volatile unsigned short             VU16;
typedef volatile unsigned int               VU32;
typedef volatile unsigned long int          VUlong;
typedef volatile unsigned long long int     VU64;

/* signed */

typedef signed char                 s8;
typedef short                       s16;
typedef int                         s32;
typedef long int                    slong;
typedef long long int               s64;

typedef signed char                 S8;
typedef short                       S16;
typedef int                         S32;
typedef long int                    Slong;
typedef long long int               S64;

typedef signed char                 int8_t;
typedef short                       int16_t;
typedef int                         int32_t;
typedef long int                    long_t;
typedef long long int               int64_t;

//typedef signed char                 CHAR;
typedef short                       SHORT;
typedef int                         INT;
typedef long int                    LONG;
typedef long long int               LLONG;

typedef signed char                 INT8S;
typedef short                       INT16S;
typedef int                         INT32S;
typedef long long                   INT64S;

//typedef volatile signed char       vs8;
//typedef volatile short             vs16;
//typedef volatile int               vs32;
typedef volatile long int          vslong;
typedef volatile long long int     vs64;

typedef volatile char              VSU8;
typedef volatile short             VSU16;
typedef volatile int               VSU32;
typedef volatile long int          VSUlong;
typedef volatile long long int     VSU64;

typedef float                       FP32;
typedef double                      FP64;

/* other */
typedef void                        VOID;

#ifndef NULL
#define NULL 0
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif


#ifndef OK
#define OK          0
#endif

#ifndef ERROR
#define ERROR       -1
#endif

#if 1
typedef enum
{
    false = 0,
    true  = 1,
} bool;
#endif

#define htons(n)		((uint16_t)(((n) & 0xff) << 8) | (((n) & 0xff00) >> 8))
#define htonl(n)		((uint32_t)( \
				(((uint32_t)(n) & (uint32_t)0x000000ffUL) << 24) | \
				(((uint32_t)(n) & (uint32_t)0x0000ff00UL) <<  8) | \
				(((uint32_t)(n) & (uint32_t)0x00ff0000UL) >>  8) | \
				(((uint32_t)(n) & (uint32_t)0xff000000UL) >> 24)))
#define ntohs(n)		htons(n)
#define ntohl(n)		htonl(n)


#define __isr__
#define __ghn_wq__
#define __tmr__

#define DISPLAY_MSG		0

//#define DISABLE			0
//#define ENABLE			1

#define MAX(a,b)		((a) >= (b) ? (a) : (b))
#define MIN(a,b)		((a) <= (b) ? (a) : (b))
#define US_SWAP(a)		((a) = (INT16U)(((INT16U)(a) >> 8) | ((INT16U)(a) << 8)))
#define mod_ceiling(a, b)	(((a) + (b) - 1) / (b))
#define mod_floor(a, b)		((a) / (b))

static __inline void split_write_two(uint8_t *addr, uint16_t two, uint8_t endian)
{
	uint8_t byte0, byte1;

	byte0 = (two & 0xff);
	byte1 = (two & 0xff00) >> 8;

	if (endian) {
		*addr = byte1;
		*(addr+1) = byte0;
	} else {
		*addr = byte0;
		*(addr+1) = byte1;
	}

	return;
}

#define split_read_two(addr, endian)			\
	((endian) ?					\
	 (uint16_t)((*(addr) << 8) | (*(addr+1))) :	\
	 (uint16_t)((*(addr)) | (*(addr+1) << 8)))

static __inline void split_write_four(uint8_t *addr, uint32_t four, uint8_t endian)
{
	uint8_t byte0, byte1, byte2, byte3;

	byte0 = (four & 0xff);
	byte1 = (four & 0xff00) >> 8;
	byte2 = (four & 0xff0000) >> 16;
	byte3 = (four & 0xff000000) >> 24;

	if (endian) {
		*addr     = byte3;
		*(addr+1) = byte2;
		*(addr+2) = byte1;
		*(addr+3) = byte0;
	} else {
		*addr     = byte0;
		*(addr+1) = byte1;
		*(addr+2) = byte2;
		*(addr+3) = byte3;
	}
	return;
}


#define split_read_four(addr, endian)					\
	((endian) ?							\
	 (uint32_t)((*(addr) << 24) | (*(addr+1) << 16) | (*(addr+2) << 8) | (*(addr+3))) : \
	 (uint32_t)((*(addr)) | (*(addr+1) << 8) | (*(addr+2) << 16) | (*(addr+3) << 24)))

#endif	/* end of _TYPES_H */
