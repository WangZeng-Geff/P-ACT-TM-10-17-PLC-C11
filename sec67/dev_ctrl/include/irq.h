/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		wdog.c
Description:	wdog control
Author:			ch
Version:		v1.0
Date:			2015/06/24
History:
*/
/************************************************************************************************/
#ifndef _IRQ_H
#define _IRQ_H
#include "types.h"

typedef enum _irq_priority_t
{
    PRI_UART0       = 0x01,
    PRI_DEF
} irq_priority_t;

typedef void (*irq_handler_fp)(void);
/*****************************************************************************************************/
/*
	Function	:
	Description	:
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern void request_irq(int32_t c_irq_idx, int32_t m_irq_idx, irq_handler_fp handler);

/*****************************************************************************************************/
/*
	Function	:
	Description	:
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern void release_irq(int32_t m_irq_idx);

/*****************************************************************************************************/
/*
	Function	:
	Description	:
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern void enable_irq(int32_t m_irq_idx, void *context);

/*****************************************************************************************************/
/*
	Function	:
	Description	:
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern void disable_irq(int32_t m_irq_idx);

#endif
