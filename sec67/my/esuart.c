/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      node.c
Description:    node control
Author:         ch
Version:        v1.0
Date:           2015/07/23
History:
*/
/************************************************************************************************/
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "uart.h"
#include "types.h"
#include "esuart.h"
#include "comfunc.h"
#include "sec_printf.h"

#include "gpio.h"
#include "task.h"
#include "sec_tick.h"
#include "sec_printf.h"
#include "softtimer.h"

static uartbuf_t esuart_rxbuf = {{0}, 0, 0, 0};
static uartbuf_t esuart_txbuf = {{0}, 0, 0, 0};

/*****************************************************************************************************/
static void on_uart1_led(struct soft_timer *st)
{
	LED_ON(LED_485);
}
static void off_uart1_led(struct soft_timer *st)
{
	LED_OFF(LED_485);
}
struct soft_timer uart1_led_on_timer =
{
    .timer_cb = on_uart1_led,
};
struct soft_timer uart1_led_off_timer =
{
    .timer_cb = off_uart1_led,
};
static void uart1_led_flicker(void)
{
    soft_timer_del(&uart1_led_on_timer);
    soft_timer_del(&uart1_led_off_timer);
    uart1_led_on_timer.expires = sec_jiffies + 1;
    uart1_led_off_timer.expires = sec_jiffies + 2;
    soft_timer_add(&uart1_led_on_timer);
    soft_timer_add(&uart1_led_off_timer);
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
static void esuart_rx_isr(u32 chan, char *c, u32 len)
{
    u32 rlen = 0, wlen = 0;
	
    _OS_ENTER_CRITICAL();
    if ((esuart_rxbuf.len + len > aMaxDbufSize) || (c == NULL))
    {
        _OS_EXIT_CRITICAL();
        return;
    }
    
    if (len == 0)
    {
		//pr_info("uart rx data, len = %d\n", esuart_rxbuf.len);
        return;
    }

    if (esuart_rxbuf.wpos + len > aMaxDbufSize)
    {
        wlen = aMaxDbufSize - esuart_rxbuf.wpos;
        memcpy(esuart_rxbuf.buf + esuart_rxbuf.wpos, c + rlen, wlen);
        rlen += wlen;
        esuart_rxbuf.wpos = 0;
    }
    memcpy(esuart_rxbuf.buf + esuart_rxbuf.wpos, c + rlen, len - wlen);
    esuart_rxbuf.wpos += len - wlen;
    esuart_rxbuf.len += len;

    _OS_EXIT_CRITICAL();
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
static u32 esuart_tx_isr(u32 chan, char *c, u32 len)
{
    u32 rlen = 0, wlen = 0, dlen = len;

    _OS_ENTER_CRITICAL();
    if ((esuart_txbuf.len == 0) || (esuart_txbuf.len + len > aMaxDbufSize) || (c == NULL) || (len == 0))
    {
        _OS_EXIT_CRITICAL();
        return 0;
    }
    
    if (esuart_txbuf.len < len)
    {
        dlen = esuart_txbuf.len;
    }
    if (esuart_txbuf.rpos + dlen > aMaxDbufSize)
    {
        rlen = aMaxDbufSize - esuart_txbuf.rpos;
        memcpy(c + wlen, esuart_txbuf.buf + esuart_txbuf.rpos, rlen);
        wlen += rlen;
        esuart_txbuf.len -= rlen;
        esuart_txbuf.rpos = 0;
    }
    memcpy(c + wlen, esuart_txbuf.buf + esuart_txbuf.rpos, dlen - rlen);
    esuart_txbuf.rpos += dlen - rlen;
    esuart_txbuf.len -= dlen;

    _OS_EXIT_CRITICAL();
    return dlen;
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
void esuart_init(void)
{
     uart_init(UART_CHN_DM, esuart_rx_isr, esuart_tx_isr);
	 uart_config(UART_CHN_DM, UART_BAUD_9600, UART_DataMode_8);
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
u32 esuart_send_data(u32 chan, u8 *data, u32 len)
{
    u32 rlen = 0, wlen = 0;

//	pr_info("uart: send data, len[%d]\n", len);
//    print_debug_array(KERN_INFO, data, len);
    _OS_ENTER_CRITICAL();
    if ((esuart_txbuf.len + len > aMaxDbufSize) || (data == NULL) || (len == 0))
    {
        _OS_EXIT_CRITICAL();
        return 0;
    }
	uart1_led_flicker();
    if (esuart_txbuf.wpos + len > aMaxDbufSize)
    {
        wlen = aMaxDbufSize - esuart_txbuf.wpos;
        memcpy(esuart_txbuf.buf + esuart_txbuf.wpos, data, wlen);
        rlen += wlen;
        esuart_txbuf.wpos = 0;
    }
    memcpy(esuart_txbuf.buf + esuart_txbuf.wpos, data + rlen, len - rlen);
    esuart_txbuf.wpos += len - rlen;
    esuart_txbuf.len += len;

    _OS_EXIT_CRITICAL();
    
    uart_ITsend_start(chan);

    return len;
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
u32 esuart_peak_data(uint32_t chn, uint8_t *buf, uint32_t len)
{
    u32 rlen = 0, wlen = 0, dlen = len, rpos = esuart_rxbuf.rpos;

    _OS_ENTER_CRITICAL();
    if ((esuart_rxbuf.len == 0) || (buf == NULL) || (len == 0))
    {
        _OS_EXIT_CRITICAL();
        return 0;
    }
	uart1_led_flicker();
    if (esuart_rxbuf.len <= len)
    {
        dlen = esuart_rxbuf.len;
    }
    if (esuart_rxbuf.rpos + dlen > aMaxDbufSize)
    {
        rlen = aMaxDbufSize - esuart_rxbuf.rpos;
        memcpy(buf + wlen, esuart_rxbuf.buf + esuart_rxbuf.rpos, rlen);
        wlen += rlen;
        rpos = 0;
    }
    memcpy(buf + wlen, esuart_rxbuf.buf + rpos, dlen - wlen);

    _OS_EXIT_CRITICAL();
	
    return dlen;
}

/*****************************************************************************************************/
/*
    Function    :
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
u32 esuart_read_data(uint32_t chn, uint8_t *buf, uint32_t len)
{
    u32 rlen = 0, wlen = 0, dlen = len;

    _OS_ENTER_CRITICAL();
    if ((esuart_rxbuf.len == 0) || (buf == NULL) || (len == 0))
    {
        _OS_EXIT_CRITICAL();
        return 0;
    }
    if (esuart_rxbuf.len <= len)
    {
        dlen = esuart_rxbuf.len;
    }
    if (esuart_rxbuf.rpos + dlen > aMaxDbufSize)
    {
        rlen = aMaxDbufSize - esuart_rxbuf.rpos;
        memcpy(buf + wlen, esuart_rxbuf.buf + esuart_rxbuf.rpos, rlen);
        wlen += rlen;
        esuart_rxbuf.rpos = 0;
    }
    memcpy(buf + wlen, esuart_rxbuf.buf + esuart_rxbuf.rpos, dlen - wlen);
    esuart_rxbuf.rpos += dlen - wlen;
    esuart_rxbuf.len -= dlen;

    _OS_EXIT_CRITICAL();
    return dlen;
}
