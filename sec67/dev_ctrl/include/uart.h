/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      uart.h
Description:    the uart interface control
Author:         ch
Version:        v1.0
Date:           2017/07/26
History:
*/
/************************************************************************************************/
#ifndef __UART_H
#define __UART_H

#include "ssc1667.h"
#include "types.h"

typedef enum
{
    UART_BAUD_600     = 600,
    UART_BAUD_1200    = 1200,
    UART_BAUD_2400    = 2400,
    UART_BAUD_4800    = 4800,
    UART_BAUD_9600    = 9600,
    UART_BAUD_14400   = 14400,
    UART_BAUD_19200   = 19200,
    UART_BAUD_38400   = 38400,
    UART_BAUD_56000   = 56000,
    UART_BAUD_57600   = 57600,
    UART_BAUD_115200  = 115200,
    UART_BAUD_128000  = 128000,
    UART_BAUD_230400  = 230400,
    UART_BAUD_256000  = 256000,
    UART_BAUD_460800  = 460800,
} UART_BAUD;

typedef enum
{
    UART_DataMode_7       = 0x4,  //7位数据
    UART_DataMode_8       = 0x0,  //8位数据
    UART_DataMode_9       = 0x2,  //9位数据
    UART_DataMode_7Odd    = 0xD,  //7位数据＋奇校验
    UART_DataMode_7Even   = 0xC,  //7位数据＋偶校验
    UART_DataMode_7Add0   = 0xE,  //7位数据＋固定0
    UART_DataMode_7Add1   = 0xF,  //7位数据＋固定1
    UART_DataMode_8Odd    = 0x9,  //8位数据＋奇校验
    UART_DataMode_8Even   = 0x8,  //8位数据＋偶校验
    UART_DataMode_8Add0   = 0xA,  //8位数据＋固定0
    UART_DataMode_8Add1   = 0xB,  //8位数据＋固定1
} UART_TYPE_DATAMOD;
#define UART_CHN_1        (0)
#define UART_CHN_DM       (0)
#define UART_CHN_DBG      (1)
#define UART_CHN_INF      (2)

typedef void (*UART_RX_ISR)(uint32_t chn, char *c, uint32_t len);
typedef uint32_t (*UART_TX_ISR)(uint32_t chn, char *c, uint32_t len);

typedef void (*UART_CONFIG_FUNC)(UART_RX_ISR, UART_TX_ISR);
/*****************************************************************************************************/
/*
    Function    : uart_init
    Description : uart init
    Input       : chn - uart chan idx [0,3]
                  rx - user rx isr
                  tx - user tx isr
    Output      : none
    Return      : none
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_init(uint32_t chn, UART_RX_ISR rx, UART_TX_ISR tx);

/*****************************************************************************************************/
/*
    Function    : uart_config_1
    Description : config uart1 as shell
    Input       : none
    Output      : none
    Return      : none
    Notes       : 115200, 8, 1, n
*/
/*****************************************************************************************************/
extern void uart_config_1(void);

/*****************************************************************************************************/
/*
    Function    : uart_config
    Description : config uartx parameter
    Input       : chn  - uart chan idx [0,3]
                  baud - baud rate
                  mod  - data mod-> UART_TYPE_DATAMOD
    Output      : 0  - config success
                  -1 - config failed
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern int32_t uart_config(uint32_t chn, UART_BAUD baud, UART_TYPE_DATAMOD mod);

/*****************************************************************************************************/
/*
    Function    : uart_send_data
    Description : uart send data
    Input       : chn  - uart chn idx
                  data - data need to be send
                  len  - data len
    Output      : none
    Return      : none
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_send_data(uint8_t chn, uint8_t *data, uint32_t len);

/*****************************************************************************************************/
/*
    Function    : uart_ITsend_start
    Description : uart send data IT enable
    Input       : chn  - uart chn idx
    Output      : none
    Return      : none
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_ITsend_start(uint32_t chn);

/*****************************************************************************************************/
/*
    Function    : uart_ITsend_stop
    Description : uart send data IT disable
    Input       : chn  - uart chn idx
    Output      : none
    Return      : none
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_ITsend_stop(uint32_t chn);

/*****************************************************************************************************/
/*
    Function    : inbyte
    Description : uart1 read a char
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern char inbyte(void);

/*****************************************************************************************************/
/*
    Function    : inbyte_wait
    Description : uart1 read a char for delay
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern char inbyte_wait(void);

/*****************************************************************************************************/
/*
    Function    : uart_putc
    Description : uart1 output char
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_putc(char c);

/*****************************************************************************************************/
/*
    Function    : uart_puts
    Description : uart1 output string
    Input       :
    Output      :
    Return      :
    Notes       : pcOut must end with a '\0'
*/
/*****************************************************************************************************/
extern void uart_puts(char *pcOut);

/*****************************************************************************************************/
/*
    Function    : uart_cmd_rx
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_cmd_rx(UART_TypeDef *UARTx, FuncState NewState);

/*****************************************************************************************************/
/*
    Function    : uart_cmd_tx
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void uart_cmd_tx(UART_TypeDef *UARTx, FuncState NewState);

#endif
