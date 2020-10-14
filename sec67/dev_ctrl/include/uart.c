/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      uart.c
Description:    the uart control
Author:         ch
Version:        v1.0
Date:           2017/07/26
History:
*/
/************************************************************************************************/
#include "types.h"
#include "uart.h"
#include "uart_reg.h"
#include "gpio.h"
#include "scu.h"
#include "irq.h"

static void uart_config_0(UART_RX_ISR rx, UART_TX_ISR tx);

static void uart_reset(uint8_t chn);

static void uart_config_ctrl(UART_TypeDef *UARTx, UART_InitStruType *UART_InitStruct);

static void uart_cmd(UART_TypeDef *UARTx, FuncState NewState);

static void __isr__ uart_0_isr(void);

static UART_RX_ISR uart_rx_isr[1] = {NULL};

static UART_TX_ISR uart_tx_isr[1] = {NULL};

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
void uart_init(uint32_t chn, UART_RX_ISR rx, UART_TX_ISR tx)
{
    UART_CONFIG_FUNC config[] = {uart_config_0};

    if (chn > sizeof(config) / sizeof(config[0]))
    {
        return;
    }
    config[chn](rx, tx);
}

/*****************************************************************************************************/
/*
    Function    : uart_config_0
    Description : config uart1 as DM
    Input       : rx - user rx isr
                  tx - user tx isr
    Output      : none
    Return      : none
    Notes       : 115200, 8, 1, n start rx
*/
/*****************************************************************************************************/
static void uart_config_0(UART_RX_ISR rx, UART_TX_ISR tx)
{
    scu_APB1PERI_ctrl(SCU_UART0, CLK_EN);

    uart_reset(0);

    gpio_set_map(GPIO_9, UART0_TXD);
    gpio_set_map(GPIO_8, UART0_RXD);
    gpio_set_dir(GPIO_9, GPIO_DIR_OUT);
    gpio_set_dir(GPIO_8, GPIO_DIR_IN);

    uart_rx_isr[0] = rx;
    uart_tx_isr[0] = tx;


    uart_cmd(UART0, DISABLE);
    uart_config(UART_CHN_DM, UART_BAUD_115200, UART_DataMode_8Even);

    release_irq(UART0_IRQn);
    request_irq(UART0_IRQn, PRI_UART0, uart_0_isr);

    UART0->CON1.IDM    = UART_IDM_Idle_3;
    UART0->CON0.IDEN   = 1;
    UART0->IE.RBIE     = 1;    /* enable RB&ID */

#ifdef DEF_ETOS
    UART0->IE.IDIE     = 1;
#endif

    uart_cmd(UART0, ENABLE);
}

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
int32_t uart_config(uint32_t chn, UART_BAUD baud, UART_TYPE_DATAMOD mod)
{
    UART_TypeDef *UARTx = (UART_TypeDef * )(_UARTx(chn));
    uint8_t dmod = (uint8_t)(mod & 0x0F);
    UART_InitStruType config;

    if ((chn > UART_MAX_INX) || (INVAL_DMODE_CHK(dmod)))
    {
        return -1;
    }

    config.UART_BaudRate   = baud;
    config.UART_ClockSet   = UART_Clock_1;
    config.UART_RxMode     = mod;
    config.UART_TxMode     = mod;
    config.UART_RxPolar    = UART_Polar_Normal;
    config.UART_TxPolar    = UART_Polar_Normal;
    config.UART_StopBits   = UART_StopBits_1;

    uart_config_ctrl(UARTx, &config);

    return 0;
}

/*****************************************************************************************************/
/*
    Function    : uart_put_char
    Description : send 1-Byte data
    Input       : none
    Output      : none
    Return      : none
    Notes       :
*/
/*****************************************************************************************************/
void uart_put_char(UART_TypeDef *UARTx, char c)
{
    _OS_ENTER_CRITICAL();
    UART_SendData(UARTx, (uint8_t)c);
    while (UART_GetIF(UARTx, UART_FLAG_TB) == RESET);
    while(UART_GetITStatus(UARTx, UART_STA_TXBUSY) != RESET)
    _OS_EXIT_CRITICAL();
}

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
void uart_send_data(uint8_t chn, uint8_t *data, uint32_t len)
{
    UART_TypeDef *UARTx = (UART_TypeDef * )(_UARTx(chn));
    uint32_t i = 0;
    for (i = 0; i < len; i++)
    {
        uart_put_char(UARTx, data[i]);
    }
}

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
void uart_ITsend_start(uint32_t chn)
{
    UART_TypeDef *UARTx = (UART_TypeDef * )(_UARTx(chn));

//    UARTx->IE.TBIE = 0;    /* 清除发送缓冲区 */
//    UARTx->IE.TCIE = 0;    /* 清除发送缓冲区 */

//    UARTx->CON0.TBCLR = 1;

    UARTx->IE.TBIE    = 1;
}

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
void uart_ITsend_stop(uint32_t chn)
{
    UART_TypeDef *UARTx = (UART_TypeDef * )(_UARTx(chn));

    UARTx->IE.TBIE = 0;    /* 清除发送缓冲区 */

    UARTx->CON0.TBCLR = 1;
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
static void __isr__ uart_0_isr(void)
{
    char c;
    uint32_t num = 0, ret = 0;

    _OS_ENTER_CRITICAL();
    if (UART_GetIF(UART0, UART_FLAG_RB) != RESET)
    {
        num = UAER_GetRXCnt(UART0);
        while (num > 0)
        {
            c = UART_ReceiveData(UART0);
            if (uart_rx_isr[0] != NULL)
            {
                uart_rx_isr[0](0, &c, 0x01);
            }
            --num;
        }
    }

    if (UART_GetIF(UART0, UART_FLAG_ID) != RESET)
    {
        UART_ClearIF(UART0, UART_FLAG_ID);
        uart_rx_isr[0](0, &c, 0x00);                /* ETOS user this irq to post sem */

    }

    if (UART_GetIF(UART0, UART_FLAG_TB) != RESET)
    {
//        while(UART_GetITStatus(UART0, UART_STA_TXBUSY) != RESET);
//        UART0->IE.TBIE    = 0;
        while (UAER_GetTXCnt(UART0) < 8)
        {
            if (uart_tx_isr[0] != NULL)
            {
                ret = uart_tx_isr[0](0, &c, 0x01);
            }
            if (ret > 0)
            {
                UART_SendData(UART0, c);
            }
            else
            {
                break;                          /* 最后一个fifo会产生接收完成中断 */
            }
            num += ret;
        }
        if (num == 0)
        {
            UART0->IE.TBIE    = 0;
        }
    }
    _OS_EXIT_CRITICAL();

}

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
void uart_cmd_rx(UART_TypeDef *UARTx, FuncState NewState)
{
    if (NewState != DISABLE)
    {
        UARTx->CON0.RXEN = 1;
    } else
    {
        UARTx->CON0.RXEN = 0;
    }
}

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
void uart_cmd_tx(UART_TypeDef *UARTx, FuncState NewState)
{
    if (NewState != DISABLE)
    {
        UARTx->CON0.TXEN = 1;
    } else
    {
        UARTx->CON0.TXEN = 0;
    }
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
static void uart_reset(uint8_t chn)
{
    UART_TypeDef *UARTx = (UART_TypeDef * )(_UARTx(chn));

    uart_cmd(UARTx, DISABLE);
    UART_ClearIF(UARTx, UART_CLR_ALL);
    UARTx->CON0.RBCLR = 1;
    UARTx->CON0.TBCLR = 1;
    uart_cmd(UARTx, ENABLE);
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
static void uart_config_ctrl(UART_TypeDef *UARTx, UART_InitStruType *UART_InitStruct)
{
    float div = 0;
    uint32_t clk;


    UARTx->CON1.BCS = UART_InitStruct->UART_ClockSet;

    if ((UART_InitStruct->UART_ClockSet > UART_Clock_4)
        || (UART_InitStruct->UART_ClockSet == UART_Clock_0))
    {
        UART_InitStruct->UART_ClockSet = UART_Clock_4;
    }

    clk = PCLK;
    div = (float)clk / (16.0 * (0x01 << (UART_InitStruct->UART_ClockSet - 1))
                        * UART_InitStruct->UART_BaudRate);


    div = (div < 1.0) ? 1.0 : div;
    UARTx->BRR.Word   = (uint32_t)(div * 16);
    UARTx->CON0.TXFS  = UART_InitStruct->UART_StopBits;
    UARTx->CON0.TXMOD = UART_InitStruct->UART_TxMode;
    UARTx->CON0.TXP   = UART_InitStruct->UART_TxPolar;
    UARTx->CON0.RXMOD = UART_InitStruct->UART_RxMode;
    UARTx->CON0.RXP   = UART_InitStruct->UART_RxPolar;
    UARTx->CON1.RBIM  = UART_TRBIM_Byte;
    UARTx->CON1.TBIM  = UART_TBIM_EMPTY;
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
static void uart_cmd(UART_TypeDef *UARTx, FuncState NewState)
{
    if (NewState != DISABLE)
    {
        UARTx->CON0.TXEN = 1;
        UARTx->CON0.RXEN = 1;
    }
    else
    {
        UARTx->CON0.TXEN = 0;
        UARTx->CON0.RXEN = 0;
    }
}


