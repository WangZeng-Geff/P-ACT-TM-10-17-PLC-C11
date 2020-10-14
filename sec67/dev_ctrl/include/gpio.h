/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      gpio.h
Description:    the gpio control
Author:         ch
Version:        v1.0
Date:           2017/07/28
History:
*/
/************************************************************************************************/

#ifndef __GPIO_H
#define __GPIO_H


#include "ssc1667.h"
#include "types.h"

/* GPIO引脚编号 */
typedef enum
{
    GPIO_0 = 0,
    GPIO_1 = 1,
    GPIO_2 = 2,
    GPIO_3 = 3,
    GPIO_4 = 4,
    GPIO_5 = 5,
    GPIO_6 = 6,
    GPIO_7 = 7,
    GPIO_8 = 8,
    GPIO_9 = 9,
    GPIO_10 = 10,
    GPIO_11 = 11,
    GPIO_12 = 12,
    GPIO_13 = 13,
    GPIO_14 = 14,
    GPIO_15 = 15,
    GPIO_16 = 16,
    GPIO_17 = 17,
    GPIO_18 = 18,
    GPIO_19 = 19,
    GPIO_20 = 20,
    GPIO_21 = 21,
    GPIO_22 = 22,
    GPIO_23 = 23,
    GPIO_24 = 24,
    GPIO_25 = 25,
    GPIO_26 = 26,
    GPIO_27 = 27,
    GPIO_28 = 28,
    GPIO_29 = 29,
    GPIO_30 = 30,
    GPIO_31 = 31,
} GPIO_PIN;

//#define DEV_RT 

#ifndef DEV_RT
#define MD_68_RT      0
#else
#define MD_68_RT      1
#endif

#define MD_MODE_1     0 /* PLCS1667(v1.01)\PLCS1667(v2.01) */
#define MD_MODE_2     0 /* PLCS1667(v1.02)\PLCT1667(v1.01)\RTB-68-GW13(V1.01) */
#define MD_MODE_3     0 /* PLCM1667-II(v1.01) */
#define MD_MODE_4     0 /* PLCM1667-II(v1.01) */
#define MD_MODE_5     0 /* ES1667-C(v1.01)\ES1667-NC(v1.01)\ES1668-C(v1.01)*/
#define MD_MODE_6     1 /* ES1667-FP(v1.01)*/

#if MD_MODE_1
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#define PLC_TX_SDN    GPIO_14  
#define PLC_ZC_PIN    GPIO_20
#define SYS_RUN_LED   PLC_TX_LED
#endif


#if MD_MODE_2
#if MD_68_RT
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#define PLC_TX_SDN    GPIO_13 
#define PLC_ZC_PIN    GPIO_14
#define SYS_RUN_LED   PLC_TX_LED
#define TXA_EN        GPIO_19
#define TXB_EN        GPIO_18
#define TXC_EN        GPIO_15
#define RXA_EN        GPIO_12
#define RXB_EN        GPIO_11
#define RXC_EN        GPIO_10
#else
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#define PLC_TX_SDN    GPIO_13 
#define PLC_ZC_PIN    GPIO_14
#define SYS_RUN_LED   PLC_TX_LED
#endif
#endif

#if MD_MODE_3
#define PLC_RX_LED    GPIO_15
#define PLC_TX_LED    GPIO_15
#define SYS_RUN_LED   GPIO_19
#define PLC_TX_SDN    GPIO_13 
#define PLC_ZC_PIN    GPIO_14
#endif

#if MD_MODE_4
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#define SYS_RUN_LED   GPIO_19
#define PLC_TX_SDN    GPIO_13 
#define PLC_ZC_PIN    GPIO_14
#endif

#if MD_MODE_5
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#define PLC_TX_SDN    GPIO_13 
#define PLC_ZC_PIN    GPIO_14
#define SYS_RUN_LED   PLC_TX_LED
#define TXA_EN        GPIO_19
#define TXB_EN        GPIO_18
#define TXC_EN        GPIO_15
#define RXA_EN        GPIO_12
#define RXB_EN        GPIO_11
#define RXC_EN        GPIO_10
#define MOD_67_68	  GPIO_3
#define MOD_LBS_NOR   GPIO_7
#endif

#if MD_MODE_6
#define DEV_KEY 		GPIO_2
#define LED_NET			GPIO_11
#define LED_485			GPIO_19
#define LED_PLC 		GPIO_18
#define PLC_RX_LED    GPIO_16
#define PLC_TX_LED    GPIO_17
#endif

#define LED_ON(led) 	gpio_out_low(led)
#define LED_OFF(led)	gpio_out_high(led)
#define LED_TOGGLE(led)	gpio_out_toggle(led)

/* 引脚复用功能选择 */
typedef enum
{
    GPIO_FUNC0 = 0,         // 引脚复用功能0
    GPIO_FUNC1 = 1,         // 引脚复用功能1
    GPIO_FUNC2 = 2,         // 引脚复用功能2
    GPIO_FUNC3 = 3,         // 引脚复用功能3
} GPIO_Reuse_Func;

/* 引脚方向选择 */
typedef enum
{
    GPIO_DIR_OUT = 0,     // 输出方向
    GPIO_DIR_IN  = 1,     // 输入方向
} GPIO_DIR;

/* 外部端口中断触发方式 */
typedef enum
{
    PINT_TRI_REDGE = 0,       // 上升沿触发中断
    PINT_TRI_TEDGE = 1,       // 下降沿触发中断
    PINT_TRI_HLEVE = 2,       // 高电平触发中断
    PINT_TRI_LLEVE = 3,       // 低电平触发中断
    PINT_TRI_BEDGE = 4,       // 上升沿和下降沿都触发中断
} PINT_TRI_STYLE;

typedef enum
{
    PWM_DIS     = 0x00,
    PWM_OUT0    = 0x01,
    PWM_OUT1    = 0x02,
    PWM_BUZ     = 0x03,
} PWM_OUT;

typedef enum
{
    PWM_TX0PS     = 0,
    PWM_TX1PS     = 8,
} PWM_TXPS;

typedef enum
{
    PWM_PLV_LOW     = 0x00,
    PWM_PLV_HIGH    = 0x01,
} PWM_PLV;

typedef enum
{
    GPIO0_FUNC  = 0x00,         // 引脚0复用功能
    GPIO1_FUNC  = 0x04,         // 引脚1复用功能
    GPIO2_FUNC  = 0x08,         // 引脚2复用功能
    GPIO3_FUNC  = 0x0C,         // 引脚3复用功能
    GPIO4_FUNC  = 0x10,         // 引脚4复用功能
    GPIO5_FUNC  = 0x14,         // 引脚5复用功能
    GPIO6_FUNC  = 0x18,         // 引脚6复用功能
    GPIO7_FUNC  = 0x1C,         // 引脚7复用功能
    GPIO8_FUNC  = 0x00,         // 引脚8复用功能
    GPIO9_FUNC  = 0x04,         // 引脚9复用功能
    GPIO10_FUNC = 0x08,         // 引脚10复用功能
    GPIO11_FUNC = 0x0C,         // 引脚11复用功能
    GPIO12_FUNC = 0x10,         // 引脚12复用功能
    GPIO13_FUNC = 0x14,         // 引脚13复用功能
    GPIO14_FUNC = 0x18,         // 引脚14复用功能
    GPIO15_FUNC = 0x1C,         // 引脚15复用功能
    GPIO16_FUNC = 0x00,         // 引脚16复用功能
    GPIO17_FUNC = 0x04,         // 引脚17复用功能
    GPIO18_FUNC = 0x08,         // 引脚18复用功能
    GPIO19_FUNC = 0x0C,         // 引脚19复用功能
    GPIO20_FUNC = 0x10,         // 引脚20复用功能
    GPIO21_FUNC = 0x14,         // 引脚21复用功能
    GPIO22_FUNC = 0x18,         // 引脚22复用功能
    GPIO23_FUNC = 0x1C,         // 引脚23复用功能
    GPIO24_FUNC = 0x00,         // 引脚24复用功能
    GPIO25_FUNC = 0x04,         // 引脚25复用功能
    GPIO26_FUNC = 0x08,         // 引脚26复用功能
    GPIO27_FUNC = 0x0C,         // 引脚27复用功能
    GPIO28_FUNC = 0x10,         // 引脚28复用功能
    GPIO29_FUNC = 0x14,         // 引脚29复用功能
    GPIO30_FUNC = 0x18,         // 引脚30复用功能
    GPIO31_FUNC = 0x1C,         // 引脚31复用功能
} GPIO_FUNC_OFF;

typedef enum
{
    GPIO0_FUNC0  = (GPIO_FUNC0 << GPIO0_FUNC),
    GPIO0_FUNC1  = (GPIO_FUNC1 << GPIO0_FUNC),
    GPIO0_FUNC2  = (GPIO_FUNC2 << GPIO0_FUNC),
    GPIO0_FUNC3  = (GPIO_FUNC3 << GPIO0_FUNC),
    GPIO1_FUNC0  = (GPIO_FUNC0 << GPIO1_FUNC),
    GPIO1_FUNC1  = (GPIO_FUNC1 << GPIO1_FUNC),
    GPIO1_FUNC2  = (GPIO_FUNC2 << GPIO1_FUNC),
    GPIO1_FUNC3  = (GPIO_FUNC3 << GPIO1_FUNC),
    GPIO2_FUNC0  = (GPIO_FUNC0 << GPIO2_FUNC),
    GPIO2_FUNC1  = (GPIO_FUNC1 << GPIO2_FUNC),
    GPIO2_FUNC2  = (GPIO_FUNC2 << GPIO2_FUNC),
    GPIO2_FUNC3  = (GPIO_FUNC3 << GPIO2_FUNC),
    GPIO3_FUNC0  = (GPIO_FUNC0 << GPIO3_FUNC),
    GPIO3_FUNC1  = (GPIO_FUNC1 << GPIO3_FUNC),
    GPIO3_FUNC2  = (GPIO_FUNC2 << GPIO3_FUNC),
    GPIO3_FUNC3  = (GPIO_FUNC3 << GPIO3_FUNC),
    GPIO4_FUNC0  = (GPIO_FUNC0 << GPIO4_FUNC),
    GPIO4_FUNC1  = (GPIO_FUNC1 << GPIO4_FUNC),
    GPIO4_FUNC2  = (GPIO_FUNC2 << GPIO4_FUNC),
    GPIO4_FUNC3  = (GPIO_FUNC3 << GPIO4_FUNC),
    GPIO5_FUNC0  = (GPIO_FUNC0 << GPIO5_FUNC),
    GPIO5_FUNC1  = (GPIO_FUNC1 << GPIO5_FUNC),
    GPIO5_FUNC2  = (GPIO_FUNC2 << GPIO5_FUNC),
    GPIO5_FUNC3  = (GPIO_FUNC3 << GPIO5_FUNC),
    GPIO6_FUNC0  = (GPIO_FUNC0 << GPIO6_FUNC),
    GPIO6_FUNC1  = (GPIO_FUNC1 << GPIO6_FUNC),
    GPIO6_FUNC2  = (GPIO_FUNC2 << GPIO6_FUNC),
    GPIO6_FUNC3  = (GPIO_FUNC3 << GPIO6_FUNC),
    GPIO7_FUNC0  = (GPIO_FUNC0 << GPIO7_FUNC),
    GPIO7_FUNC1  = (GPIO_FUNC1 << GPIO7_FUNC),
    GPIO7_FUNC2  = (GPIO_FUNC2 << GPIO7_FUNC),
    GPIO7_FUNC3  = (GPIO_FUNC3 << GPIO7_FUNC),
    GPIO8_FUNC0  = (GPIO_FUNC0 << GPIO8_FUNC),
    GPIO8_FUNC1  = (GPIO_FUNC1 << GPIO8_FUNC),
    GPIO8_FUNC2  = (GPIO_FUNC2 << GPIO8_FUNC),
    GPIO8_FUNC3  = (GPIO_FUNC3 << GPIO8_FUNC),
    GPIO9_FUNC0  = (GPIO_FUNC0 << GPIO9_FUNC),
    GPIO9_FUNC1  = (GPIO_FUNC1 << GPIO9_FUNC),
    GPIO9_FUNC2  = (GPIO_FUNC2 << GPIO9_FUNC),
    GPIO9_FUNC3  = (GPIO_FUNC3 << GPIO9_FUNC),
    GPIO10_FUNC0 = (GPIO_FUNC0 << GPIO10_FUNC),
    GPIO10_FUNC1 = (GPIO_FUNC1 << GPIO10_FUNC),
    GPIO10_FUNC2 = (GPIO_FUNC2 << GPIO10_FUNC),
    GPIO10_FUNC3 = (GPIO_FUNC3 << GPIO10_FUNC),
    GPIO11_FUNC0 = (GPIO_FUNC0 << GPIO11_FUNC),
    GPIO11_FUNC1 = (GPIO_FUNC1 << GPIO11_FUNC),
    GPIO11_FUNC2 = (GPIO_FUNC2 << GPIO11_FUNC),
    GPIO11_FUNC3 = (GPIO_FUNC3 << GPIO11_FUNC),
    GPIO12_FUNC0 = (GPIO_FUNC0 << GPIO12_FUNC),
    GPIO12_FUNC1 = (GPIO_FUNC1 << GPIO12_FUNC),
    GPIO12_FUNC2 = (GPIO_FUNC2 << GPIO12_FUNC),
    GPIO12_FUNC3 = (GPIO_FUNC3 << GPIO12_FUNC),
    GPIO13_FUNC0 = (GPIO_FUNC0 << GPIO13_FUNC),
    GPIO13_FUNC1 = (GPIO_FUNC1 << GPIO13_FUNC),
    GPIO13_FUNC2 = (GPIO_FUNC2 << GPIO13_FUNC),
    GPIO13_FUNC3 = (GPIO_FUNC3 << GPIO13_FUNC),
    GPIO14_FUNC0 = (GPIO_FUNC0 << GPIO14_FUNC),
    GPIO14_FUNC1 = (GPIO_FUNC1 << GPIO14_FUNC),
    GPIO14_FUNC2 = (GPIO_FUNC2 << GPIO14_FUNC),
    GPIO14_FUNC3 = (GPIO_FUNC3 << GPIO14_FUNC),
    GPIO15_FUNC0 = (GPIO_FUNC0 << GPIO15_FUNC),
    GPIO15_FUNC1 = (GPIO_FUNC1 << GPIO15_FUNC),
    GPIO15_FUNC2 = (GPIO_FUNC2 << GPIO15_FUNC),
    GPIO15_FUNC3 = (GPIO_FUNC3 << GPIO15_FUNC),
    GPIO16_FUNC0 = (GPIO_FUNC0 << GPIO16_FUNC),
    GPIO16_FUNC1 = (GPIO_FUNC1 << GPIO16_FUNC),
    GPIO16_FUNC2 = (GPIO_FUNC2 << GPIO16_FUNC),
    GPIO16_FUNC3 = (GPIO_FUNC3 << GPIO16_FUNC),
    GPIO17_FUNC0 = (GPIO_FUNC0 << GPIO17_FUNC),
    GPIO17_FUNC1 = (GPIO_FUNC1 << GPIO17_FUNC),
    GPIO17_FUNC2 = (GPIO_FUNC2 << GPIO17_FUNC),
    GPIO17_FUNC3 = (GPIO_FUNC3 << GPIO17_FUNC),
    GPIO18_FUNC0 = (GPIO_FUNC0 << GPIO18_FUNC),
    GPIO18_FUNC1 = (GPIO_FUNC1 << GPIO18_FUNC),
    GPIO18_FUNC2 = (GPIO_FUNC2 << GPIO18_FUNC),
    GPIO18_FUNC3 = (GPIO_FUNC3 << GPIO18_FUNC),
    GPIO19_FUNC0 = (GPIO_FUNC0 << GPIO19_FUNC),
    GPIO19_FUNC1 = (GPIO_FUNC1 << GPIO19_FUNC),
    GPIO19_FUNC2 = (GPIO_FUNC2 << GPIO19_FUNC),
    GPIO19_FUNC3 = (GPIO_FUNC3 << GPIO19_FUNC),
    GPIO20_FUNC0 = (GPIO_FUNC0 << GPIO20_FUNC),
    GPIO20_FUNC1 = (GPIO_FUNC1 << GPIO20_FUNC),
    GPIO20_FUNC2 = (GPIO_FUNC2 << GPIO20_FUNC),
    GPIO20_FUNC3 = (GPIO_FUNC3 << GPIO20_FUNC),
    GPIO21_FUNC0 = (GPIO_FUNC0 << GPIO21_FUNC),
    GPIO21_FUNC1 = (GPIO_FUNC1 << GPIO21_FUNC),
    GPIO21_FUNC2 = (GPIO_FUNC2 << GPIO21_FUNC),
    GPIO21_FUNC3 = (GPIO_FUNC3 << GPIO21_FUNC),
    GPIO22_FUNC0 = (GPIO_FUNC0 << GPIO22_FUNC),
    GPIO22_FUNC1 = (GPIO_FUNC1 << GPIO22_FUNC),
    GPIO22_FUNC2 = (GPIO_FUNC2 << GPIO22_FUNC),
    GPIO22_FUNC3 = (GPIO_FUNC3 << GPIO22_FUNC),
    GPIO23_FUNC0 = (GPIO_FUNC0 << GPIO23_FUNC),
    GPIO23_FUNC1 = (GPIO_FUNC1 << GPIO23_FUNC),
    GPIO23_FUNC2 = (GPIO_FUNC2 << GPIO23_FUNC),
    GPIO23_FUNC3 = (GPIO_FUNC3 << GPIO23_FUNC),
    GPIO24_FUNC0 = (GPIO_FUNC0 << GPIO24_FUNC),
    GPIO24_FUNC1 = (GPIO_FUNC1 << GPIO24_FUNC),
    GPIO24_FUNC2 = (GPIO_FUNC2 << GPIO24_FUNC),
    GPIO24_FUNC3 = (GPIO_FUNC3 << GPIO24_FUNC),
    GPIO25_FUNC0 = (GPIO_FUNC0 << GPIO25_FUNC),
    GPIO25_FUNC1 = (GPIO_FUNC1 << GPIO25_FUNC),
    GPIO25_FUNC2 = (GPIO_FUNC2 << GPIO25_FUNC),
    GPIO25_FUNC3 = (GPIO_FUNC3 << GPIO25_FUNC),
    GPIO26_FUNC0 = (GPIO_FUNC0 << GPIO26_FUNC),
    GPIO26_FUNC1 = (GPIO_FUNC1 << GPIO26_FUNC),
    GPIO26_FUNC2 = (GPIO_FUNC2 << GPIO26_FUNC),
    GPIO26_FUNC3 = (GPIO_FUNC3 << GPIO26_FUNC),
    GPIO27_FUNC0 = (GPIO_FUNC0 << GPIO27_FUNC),
    GPIO27_FUNC1 = (GPIO_FUNC1 << GPIO27_FUNC),
    GPIO27_FUNC2 = (GPIO_FUNC2 << GPIO27_FUNC),
    GPIO27_FUNC3 = (GPIO_FUNC3 << GPIO27_FUNC),
    GPIO28_FUNC0 = (GPIO_FUNC0 << GPIO28_FUNC),
    GPIO28_FUNC1 = (GPIO_FUNC1 << GPIO28_FUNC),
    GPIO28_FUNC2 = (GPIO_FUNC2 << GPIO28_FUNC),
    GPIO28_FUNC3 = (GPIO_FUNC3 << GPIO28_FUNC),
    GPIO29_FUNC0 = (GPIO_FUNC0 << GPIO29_FUNC),
    GPIO29_FUNC1 = (GPIO_FUNC1 << GPIO29_FUNC),
    GPIO29_FUNC2 = (GPIO_FUNC2 << GPIO29_FUNC),
    GPIO29_FUNC3 = (GPIO_FUNC3 << GPIO29_FUNC),
    GPIO30_FUNC0 = (GPIO_FUNC0 << GPIO30_FUNC),
    GPIO30_FUNC1 = (GPIO_FUNC1 << GPIO30_FUNC),
    GPIO30_FUNC2 = (GPIO_FUNC2 << GPIO30_FUNC),
    GPIO30_FUNC3 = (GPIO_FUNC3 << GPIO30_FUNC),
    GPIO31_FUNC0 = (GPIO_FUNC0 << GPIO31_FUNC),
    GPIO31_FUNC1 = (GPIO_FUNC1 << GPIO31_FUNC),
    GPIO31_FUNC2 = (GPIO_FUNC2 << GPIO31_FUNC),
    GPIO31_FUNC3 = (GPIO_FUNC3 << GPIO31_FUNC),
} GPIOx_FUNCx_TYPE;

typedef enum
{
    UART1_RXD    = GPIO22_FUNC2,
    UART1_TXD    = GPIO23_FUNC2,
    UART0_RXD    = GPIO8_FUNC1,
    UART0_TXD    = GPIO9_FUNC1,
    UART2_RXD    = GPIO16_FUNC2,
    UART2_TXD    = GPIO17_FUNC2,
    INF_38K      = GPIO20_FUNC0,
    RMII_TXD1    = GPIO31_FUNC2,
    RMII_TXD0    = GPIO30_FUNC2,
    RMII_TXEN    = GPIO29_FUNC2,
    RMII_RXD1    = GPIO28_FUNC2,
    RMII_RXD0    = GPIO27_FUNC2,
    RMII_CRSDV   = GPIO26_FUNC2,
    RMII_RXER    = GPIO25_FUNC2,
    RMII_REFCLK  = GPIO24_FUNC2,
    RMII_MDIO    = GPIO21_FUNC2,
    RMII_MDC     = GPIO20_FUNC2,
} GPIO_FUNC_MAP;

/* GPIO 使用情况 */
#define UART0_RX   GPIO_8
#define UART0_TX   GPIO_9
#define UART1_RX   GPIO_22
#define UART1_TX   GPIO_23

#define INF_38K_OUT  (GPIO_20)
#define PLC_TXLED    (GPIO_17)
#define PLC_RXLED    (GPIO_16)

extern void gpio_set_map(GPIO_PIN pin, GPIO_FUNC_MAP func);

extern void gpio_set_dir(GPIO_PIN pin, GPIO_DIR dir);

extern void gpio_set_pwm0(PWM_OUT out, PWM_PLV plv);

extern void gpio_set_pwm0(PWM_OUT out, PWM_PLV plv);

extern uint32_t gpio_input(GPIO_PIN pin);

extern void gpio_out(GPIO_PIN pin, uint32_t bit);

extern void gpio_out_high(GPIO_PIN pin);

extern void gpio_out_low(GPIO_PIN pin);

extern void gpio_toggle(GPIO_PIN pin);


#endif /* __LIB_GPIO_HGPIO6_FUNC2  */

