/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		include.h
Description:	include head files
Author:			ch
Version:		v1.0
Date:			2015/06/02
History:
*/
/************************************************************************************************/

#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "types.h"


//#ifndef MONITOR_63
//#define MONITOR_63
//#endif

//#ifndef MT_SWITCHON
//#define MT_SWITCHON
//#endif

//#ifndef TRANS_VER
//#define TRANS_VER
//#endif

#if (!((ESCONFINFO >> 4) & 0x01))
#define SSC1663_DRV
#endif

#if 1
#ifndef LWIP_SWITOFF
#define LWIP_SWITOFF
#endif
#endif

#if (((ESCONFINFO >> 4) & 0x01))
#ifndef BRIDGE_64
#define BRIDGE_64											/* ???64??? */
#endif
#endif

#ifndef DHCP_INITs
#define DHCP_INITs											/* 63/64????DHCP */
#endif

#ifndef __ESTMR__
#define __ESTMR__
#endif

#endif
