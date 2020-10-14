/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      scu.h
Description:    the scu interface
Author:         ch
Version:        v1.0
Date:           2017/07/28
History:
*/
/************************************************************************************************/


#ifndef __SCU_H
#define __SCU_H


#include "ssc1667.h"
#include "gpio.h"
#include "types.h"

/* PWRC复位状态寄存器标志位 */
typedef enum
{
  SCU_PWRC_CHIPRSTF   = (1 << 1),     //软复位CHIPRST复位标志
  SCU_PWRC_PORF_3     = (1 << 2),     //POR3.3V复位标志
  SCU_PWRC_RC_PORF    = (1 << 3),     //RC_POR复位标志
  SCU_PWRC_BORF       = (1 << 4),     //BOR复位标志
  SCU_PWRC_IWDTRSTF   = (1 << 5),     //IWDT复位标志
  SCU_PWRC_MRSTF      = (1 << 6),     //MRSTn丢失标志位
  SCU_PWRC_SOFTRSTF   = (1 << 7),     //软件丢失标志位
  SCU_PWRC_POR_LOST   = (1 << 8),     //POR丢失标志位
  SCU_PWRC_LOCKUP_RST = (1 << 9),     //CPU锁定复位标志位
  SCU_PWRC_WWDTRSTF   = (1 << 10),    //WWDT复位标志位
  SCU_PWRC_ECC        = (1 << 11),    //ECC解码错误
} SCU_TYPE_PWRC;

#define RST_IF_ALL     (0x7FE)

/*DPLL2标志位*/
typedef enum
{
  SCU_DPLL2_LOCKED  = (1 << 28), //DPLL2 LOCK 指示位
  SCU_DPLL2_FLAG1   = (1 << 29), //DPLL2数字控制输入全1指示标志
  SCU_DPLL2_FLAG0   = (1 << 30), //DPLL2数字控制输入全0指示标志
} SCU_TYPE_DPLL2;

/* 外设时钟 */
typedef enum
{
  SCU_GPIO = 0,
  SCU_FLASH = 1,
  SCU_ADC = 2,
  SCU_IWDT = 3,
  SCU_T16N0 = 4,
  SCU_T16N1 = 5,
  SCU_T16N2 = 6,
  SCU_T16N3 = 7,
  SCU_T32N0 = 8,
  SCU_T32N1 = 9,
  SCU_T32N2 = 10,
  SCU_T32N3 = 11,
  SCU_UART0 = 12,
  SCU_UART1 = 13,
  SCU_UART2 = 14,
  SCU_UART3 = 15,
  SCU_SPI0 = 16,
  SCU_SPI1 = 17,
  SCU_I2C0 = 18,
  SCU_I2C1 = 19,
  SCU_TENSOR = 20,
  SCU_ECC = 23,
  SCU_CRC = 24,
  SCU_WWDT = 25,
  SCU_OPAMP = 26,
  SCU_COMP = 27,
  SCU_RNG = 29,
  SCU_SPI2 = 30,
  SCU_SPI3 = 31,
}SCU_APB1PERI_Typedef;

typedef enum
{
  SCU_AES = 0,
  SCU_BPLC = 1,
  SCU_150M = 2,
  SCU_TIMER = 3,
}SCU_APB2PERI_Typedef;

typedef enum
{
  SCU_DMA = 0,
  SCU_USB = 1,
  SCU_SRAM = 2,
  SCU_APB1 = 3,
  SCU_APB2 = 4,
  SCU_ETH = 5,
} SCU_AHBPERI_Typedef;

typedef enum
{
  SCU_AHBEN_ABP2_PRE1_HCLK_0 = 0,
  SCU_AHBEN_ABP2_PRE1_HCLK_2,
  SCU_AHBEN_ABP2_PRE1_HCLK_4,
  SCU_AHBEN_ABP2_PRE1_HCLK_8,
}SCU_AHBEN_ABP2_PRE1_Typedef;

typedef enum
{
  SCU_AHBEN_ABP1_PRE1_HCLK_2_0 = 0,
  SCU_AHBEN_ABP1_PRE1_HCLK_2_1 = 1,
  SCU_AHBEN_ABP1_PRE1_HCLK_4   = 2,
  SCU_AHBEN_ABP1_PRE1_HCLK_8   = 3,
}SCU_AHBEN_ABP1_PRE1_Typedef;

// PLL时钟源选择
typedef enum
{
  SCU_DPLL2_HRC   = 0x0,    //PLL时钟源为内部25MHz
  SCU_DPLL2_XTAL  = 0x1,    //PLL时钟源为外部25MHz
} SCU_DPLL2_Origin;

// PLL时钟输出频率旋择
typedef enum
{
  SCU_DPLL2_200M  = 0x0 ,   //PLL时钟输出为200MHz
  SCU_DPLL2_150M  = 0x1 ,   //PLL时钟输出为150Mhz
} SCU_DPLL2_Out;

typedef enum
{
    LVD_RISING        = 0x00,
    LVD_FAILING       = 0x01,
    LVD_HIGH          = 0x02,
    LVD_LOW           = 0x03,
    LVD_RVS           = 0x04,
} SCU_LVD_TRRIGER;

typedef enum
{
    LVD_VS_180          = 0x00,
    LVD_VS_188          = 0x01,
    LVD_VS_196          = 0x02,
    LVD_VS_204          = 0x03,
    LVD_VS_212          = 0x04,
    LVD_VS_220          = 0x05,
    LVD_VS_228          = 0x06,
    LVD_VS_236          = 0x07,
    LVD_VS_244          = 0x08,
    LVD_VS_252          = 0x09,
    LVD_VS_260          = 0x0A,
    LVD_VS_268          = 0x0B,
    LVD_VS_276          = 0x0C,
    LVD_VS_284          = 0x0D,
    LVD_VS_292          = 0x0E,
    LVD_VS_300          = 0x0F,
} SCU_VS;

typedef enum
{
    CLK_EN          = 0x01,
    CLK_DIS         = 0x00,
} SCU_CLK_CTRL;

/*****************************************************************************************************/
/*
    Function    : scu_init
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void scu_init(void);

/*****************************************************************************************************/
/*
    Function    : scu_APB1PERI_ctrl
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void scu_APB1PERI_ctrl(SCU_APB1PERI_Typedef peri, SCU_CLK_CTRL ctrl);

/*****************************************************************************************************/
/*
    Function    : scu_set_lvd
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void scu_set_lvd(SCU_LVD_TRRIGER tri, SCU_VS vs);

/*****************************************************************************************************/
/*
    Function    : scu_get_rstinfo
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern uint32_t scu_get_rstinfo(void);

/*****************************************************************************************************/
/*
    Function    : scu_clr_rstinfo
    Description :
    Input       :
    Output      :
    Return      :
    Notes       :
*/
/*****************************************************************************************************/
extern void scu_clr_rstinfo(void);

#endif /* __LIB_SCU_H */
