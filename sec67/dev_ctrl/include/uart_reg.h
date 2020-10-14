/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      uart_reg.h
Description:    the uart control
Author:         ch
Version:        v1.0
Date:           2017/08/01
History:
*/
/************************************************************************************************/
#ifndef __UART_REG_H
#define __UART_REG_H

#include "ssc1667.h"
#include "types.h"

/* 停止位 */
typedef enum
{
    UART_StopBits_1 = 0x0,        //1位停止位
    UART_StopBits_2 = 0x1,        //2位停止位
} UART_TYPE_TXFS;

/* 端口极性 */
typedef enum
{
    UART_Polar_Normal    = 0,    //正极性
    UART_Polar_Opposite  = 1,    //负极性
} UART_TYPE_RTXP;

/* 时钟选择 */
typedef enum
{
    UART_Clock_0 = 0x0,         //close
    UART_Clock_1 = 0x1,         //PCLK
    UART_Clock_2 = 0x2,         //PCLK / 2
    UART_Clock_3 = 0x3,         //PCLK / 4
    UART_Clock_4 = 0x4,         //PCLK / 8
} UART_TYPE_BCS;

/* 初始化结构体 */
typedef struct
{
    UART_TYPE_TXFS     UART_StopBits;    //停止位选择
    UART_TYPE_DATAMOD  UART_TxMode;      //发送数据格式
    UART_TYPE_RTXP     UART_TxPolar;     //发送端口极性
    UART_TYPE_DATAMOD  UART_RxMode;      //接收数据格式
    UART_TYPE_RTXP     UART_RxPolar;     //接收端口极性
    uint32_t           UART_BaudRate;    //波特率
    UART_TYPE_BCS      UART_ClockSet;    //UART时钟选择
} UART_InitStruType;

/* 空闲帧检测模式 */
typedef enum
{
    UART_IDM_Idle_1 = 0x0,          //连续10个bit空闲
    UART_IDM_Idle_2 = 0x1,          //连续11个bit空闲
    UART_IDM_Idle_3 = 0x2,          //连续12个bit空闲
    UART_IDM_Idle_4 = 0x3,          //连续13个bit空闲
} UART_TYPE_IDM;

/* 自动波特率模式 */
typedef enum
{
    UART_BDM_Mode_1 = 0x0,          //接收数据最低位为1B，检测起始位波特率
    UART_BDM_Mode_2 = 0x1,          //接收数据最低两位为01B，检测起始位及数据第一位波特率
    UART_BDM_Mode_3 = 0x2,          //接收数据7FH，检测起始位及前7个数据位
    UART_BDM_Mode_4 = 0x3,          //接收数据55H，检测起始位及前7个数据位
} UART_TYPE_BDM;

/* Interrupt */
typedef enum
{
    UART_IT_TB    = (1 << 0),        //发送缓冲器空中断
    UART_IT_TC    = (1 << 1),        //发送完成中断
    UART_IT_TBWE  = (1 << 8),        //发送缓冲错误中断
    UART_IT_TBWO  = (1 << 9),        //发送缓冲溢出中断
    UART_IT_RB    = (1 << 16),       //接收缓冲器满中断
    UART_IT_ID    = (1 << 17),       //接收空闲帧中断
    UART_IT_RO    = (1 << 24),       //接收数据溢出中断
    UART_IT_FE    = (1 << 25),       //接收帧错误中断
    UART_IT_PE    = (1 << 26),       //接收校验错误中断
    UART_IT_BDE   = (1 << 27),       //波特率检测错误中断
    UART_IT_RBRE  = (1 << 28),       //读接收缓冲错误中断
    UART_IT_RBRO  = (1 << 29),       //读接收缓冲溢出中断
} UART_TYPE_IT;

/* Interrupt Flag */
typedef enum
{
    UART_FLAG_TB   = (1 << 0),      //发送缓冲器空中断
    UART_FLAG_TC   = (1 << 1),      //发送完成中断
    UART_FLAG_TBWE = (1 << 8),      //发送缓冲错误中断
    UART_FLAG_TBWO = (1 << 9),      //发送缓冲溢出中断
    UART_FLAG_RB   = (1 << 16),     //接收缓冲器满中断
    UART_FLAG_ID   = (1 << 17),     //接收空闲帧中断
    UART_FLAG_RO   = (1 << 24),     //接收数据溢出中断
    UART_FLAG_FE   = (1 << 25),     //接收帧错误中断
    UART_FLAG_PE   = (1 << 26),     //接收校验错误中断
    UART_FLAG_BDE  = (1 << 27),     //波特率检测错误中断
    UART_FLAG_RBRE = (1 << 28),     //读接收缓冲错误中断
    UART_FLAG_RBRO = (1 << 29),     //读接收缓冲溢出中断
} UART_TYPE_FLAG;

/* Interrupt Clear Flag */
typedef enum
{
    UART_CLR_TC    = (1 << 1),       //发送完成中断
    UART_CLR_TBWE  = (1 << 8),       //发送缓冲错误中断
    UART_CLR_TBWO  = (1 << 9),       //发送缓冲溢出中断
    UART_CLR_ID    = (1 << 17),      //接收空闲帧中断
    UART_CLR_RO    = (1 << 24),      //接收数据溢出中断
    UART_CLR_FE    = (1 << 25),      //接收帧错误中断
    UART_CLR_PE    = (1 << 26),      //接收校验错误中断
    UART_CLR_BDE   = (1 << 27),      //波特率检测错误中断
    UART_CLR_RBRE  = (1 << 28),      //读接收缓冲错误中断
    UART_CLR_RBRO  = (1 << 29),      //读接收缓冲溢出中断
} UART_CLR_IF;

#define UART_CLR_ALL  (0xFFFFFFFF)     /* 清除全部中断标志位 */
/* Interrupt Mode */
typedef enum
{
    UART_TRBIM_Byte     = 0x0,       //字节中断
    UART_TRBIM_HalfWord = 0x1,       //半字中断
    UART_TRBIM_Word     = 0x2,       //字中断
    UART_RBIM_Full      = 0x3,       //全中断
    UART_TBIM_EMPTY     = 0x3,       //全中断
} UART_TYPE_TRBIM;

/* Status */
typedef enum
{
    UART_STA_TBPTR     = (0x0F << 0),  /* 发送缓冲区字节数 */
    UART_STA_TBOV      = (1 << 4),     //发送缓冲器溢出状态位
    UART_STA_TXBUSY    = (1 << 5),     //发送状态位
    UART_STA_RBPTR     = (0x0F << 8),  /* 接收缓冲区字节数 */
    UART_STA_RBOV      = (1 << 12),    //接收缓冲器溢出状态位
    UART_STA_RXBUSY    = (1 << 13),    //接收状态位
    UART_STA_FER0      = (1 << 16),    //当前读取BYTE0帧格式错误位
    UART_STA_PER0      = (1 << 17),    //当前读取BYTE0校验错误位
    UART_STA_FER1      = (1 << 18),    //当前读取BYTE1帧格式错误位
    UART_STA_PER1      = (1 << 19),    //当前读取BYTE1校验错误位
    UART_STA_FER2      = (1 << 20),    //当前读取BYTE2帧格式错误位
    UART_STA_PER2      = (1 << 21),    //当前读取BYTE2校验错误位
    UART_STA_FER3      = (1 << 22),    //当前读取BYTE3帧格式错误位
    UART_STA_PER3      = (1 << 23),    //当前读取BYTE3校验错误位
} UART_TYPE_STA;

#define UART_RXCNT_OFFSET  (8)
#define UART_TXCNT_OFFSET  (0)

#define _UARTx(chn)   (UART0_BASE + ((chn) * (0x400)))

#define UART_MAX_INX      (3)
#define UART_DMode_Mask   (0x0F)

#define INVAL_DMODE_CHK(mode)   ((mode == 0x01) || (mode == 0x03)\
                                 ||(mode == 0x05) || (mode == 0x07))

#define UART_GetITStatus(UARTx, flag) (((UARTx)->STA.Word) & flag)
#define UAER_GetRXCnt(UARTx)          ((((UARTx)->STA.Word) & UART_STA_RBPTR) >> UART_RXCNT_OFFSET)
#define UAER_GetTXCnt(UARTx)          ((((UARTx)->STA.Word) & UART_STA_TBPTR) >> UART_TXCNT_OFFSET)
#define UART_ReceiveData(UARTx)       ((UARTx)->RBR.Byte[0])
#define UART_SendData(UARTx, c)       ((UARTx)->TBW.Byte[0] = c)
#define UART_ClearIF(UARTx, flag)     ((UARTx)->IF.Word = (flag))
#define UART_GetIF(UARTx, flag)       ((UARTx)->IF.Word & (flag))

#endif
