/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		node.h
Description:	node control
Author:			ch
Version:		v1.0
Date:			2015/07/23
History:
*/
/************************************************************************************************/

#ifndef _ESUART_H_
#define _ESUART_H_

#define aMaxUbufSize				(2 * (2048 + 32 + 20))		/* 最大串口接收buf长度 */
#define aMaxDbufSize				(2048 + 32 + 20)			/* 最大串口接收buf长度 */

typedef struct
{
	char buf[aMaxDbufSize];
	u32  wpos;
	u32  rpos;
	u32  len;
} uartbuf_t; 

void esuart_init(void);
u32 esuart_send_data(u32 chan, u8 *data, u32 len);
u32 esuart_peak_data(uint32_t chn, uint8_t *buf, uint32_t len);
u32 esuart_read_data(uint32_t chn, uint8_t *buf, uint32_t len);

#endif
