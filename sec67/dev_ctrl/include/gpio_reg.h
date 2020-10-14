/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      gpio_reg.h
Description:    the gpio control
Author:         ch
Version:        v1.0
Date:           2017/08/03
History:
*/
/************************************************************************************************/

#ifndef __GPIO_REG_H
#define __GPIO_REG_H


#include "ssc1667.h"
#include "types.h"


/* 引脚输出开漏使能位 */
typedef enum
{
    GPIO_ODE_OUT_DIS = 0,   // 开漏禁止
    GPIO_ODE_OUT_EN  = 1,   // 开漏使能
} GPIO_ODE_Output;

/* 引脚输出电流驱动能力选择 */
typedef enum
{
    GPIO_DS_OUT_NOMAL  = 0,    // 普通电流输出
    GPIO_DS_OUT_STRONG = 1,    // 强电流输出
} GPIO_DS_Output;

/* 引脚输入弱上拉使能 */
typedef enum
{
    GPIO_PUE_IN_DIS = 0,   // 弱上拉禁止
    GPIO_PUE_IN_EN = 1,    // 弱上拉使能
} GPIO_PUE_Input;

/* 引脚输入弱下拉使能 */
typedef enum
{
    GPIO_PDE_IN_DIS = 0,    // 弱下拉禁止
    GPIO_PDE_IN_EN  = 1,    // 弱下拉使能
} GPIO_PDE_Input;

/* 引脚信号类型 */
typedef enum
{
    GPIO_SIG_DIG  = 0,  // 数字信号引脚
    GPIO_SIG_ANA  = 1,   // 模拟信号引脚
} GPIO_Pin_Signal;

/* GPIO初始化配置结构定义 */
typedef struct
{
    GPIO_Pin_Signal Signal;       // 引脚上的信号类型，只有模拟和数字两种

    GPIO_DIR        Dir;          // 数字引脚时，引脚输入、输出方向选择
    GPIO_Reuse_Func Func;         // 数字引脚时，引脚复用功能选择

    GPIO_ODE_Output ODE_Output;   // 数字引脚、输出方向时，开漏输出选择
    GPIO_DS_Output DS_Output;     // 数字引脚、输出方向时，电流输出大小选择

    GPIO_PUE_Input PUE_Input;     // 数字引脚、输入方向时，弱上拉选择
    GPIO_PDE_Input PDE_Input;     // 数字引脚、输入方向时，弱下拉选择
} GPIO_InitSettingType;

/* 外部端口中断使能 */
typedef enum
{
    PINT_IE_DIS = 0,              // 中断禁止
    PINT_IE_EN  = 1,              // 中断使能
} PINT_IE_Set;

/* KINT中断使能 */
typedef enum
{
    KINT_IE_Set_Disable = 0,          // 中断禁止
    KINT_IE_Set_Enable = 1,           // 中断使能
} KINT_IE_Set;

/* KINT中断方式 */
typedef enum
{
    KINT_Trigger_Rising_Edge = 0,    // 上升沿触发中断
    KINT_Trigger_Trailing_Edge = 1,  // 下降沿触发中断
    KINT_Trigger_High_Level = 2,     // 高电平触发中断
    KINT_Trigger_Low_Level = 3,      // 低电平触发中断
    KINT_Trigger_Both_Edge = 4,      // 上升沿和下降沿都触发中断
} KINT_Trigger_Style;

/* KINT初始化配置结构定义 */
typedef struct
{
    KINT_IE_Set IE_Set;               // 中断是否使能
    KINT_Trigger_Style Trigger_Style; // 中断触发方式
} KINT_InitSettingType;

typedef enum
{
    GPIO_PINT0          = 0x00,
    GPIO_PINT1          = 0x01,
    GPIO_PINT2          = 0x02,
    GPIO_PINT3          = 0x03,
    GPIO_PINT4          = 0x04,
    GPIO_PINT5          = 0x05,
    GPIO_PINT6          = 0x06,
    GPIO_PINT7          = 0x07,
} GPIO_PINT;

#define GPIO_IRQn_ENUM   GPIO0_IRQn, GPIO1_IRQn, GPIO2_IRQn, GPIO3_IRQn, GPIO4_IRQn, GPIO5_IRQn, GPIO6_IRQn, GPIO7_IRQn

#define GPIO_PINT_MAX                   (8)

#define GPIO_PINTx(pin)                 ((pin) & 0x07)
#define GPIOx_FUNCx(pin, func)          (func << (GPIO_PINTx(pin) * 4))

#define GPIO_PINT_CLRMASK(pint)         (GPIO->PINTIE.PMASK_V &= ~(0x01 << (pint)))
#define GPIO_PINTEN(pint)               (GPIO->PINTIE.Word |= (0x01 << (pint)))
#define GPIO_PINTDIS(pint)              (GPIO->PINTIE.Word &= (~(0x01 << (pint))))

#define GPIO_GETIF(pint)                (GPIO->PINTIF.Word &= (0x01 << (pint)))
#define GPIO_CLRIF(pint)                (GPIO->PINTIF.Word = (0x01 << (pint)))

#define GPIO_SELx(pin)                  (((pin) & 0x1F) >> 3)
#define GPIO_SETSEL(pin)                (GPIO->PINTSEL.Word |= GPIO_SELx(pin) << (GPIO_PINTx(pin) * 4))

#define GPIO_SETTRI(pint, tri)           (GPIO->PINTCFG.Word |= (tri) << ((pint) * 4))


#endif /* __GPIO_reg  */

