/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      ssc1667.h
Description:    the 67&68 register & base address
Author:         ch
Version:        v1.0
Date:           2017/07/25
History:
*/
/************************************************************************************************/

#ifndef __SSC1667_H
#define __SSC1667_H


#define __I volatile const       /* defines 'read only' permissions */
#define __O volatile             /* defines 'write only' permissions */
#define __IO volatile            /* defines 'read / write' permissions */

#define __CM0_REV 0              /* Core Revision r0p0 */
#define __NVIC_PRIO_BITS 3       /* HR8P506 uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0 /* Set to 1 if different SysTick Config is used */

#define CPU_CLK  (150000000)    /*150MHz*/
//#define CPU_CLK  (200000000)    /*200MHz*/

#define CPU_CORE_CLOCK_MHZ        150

typedef enum IRQn
{
  /****** Cortex-M3 Processor Exceptions Numbers ******************************************************/
  RST_IRQn                   = -15,
  NonMaskableInt_IRQn        = -14,
  HardFault_IRQn             = -13,
  MemoryManagement_IRQn      = -12,
  BusFault_IRQn              = -11,
  UsageFault_IRQn            = -10,
  SVC_IRQn                   = -5,
  PendSV_IRQn                = -2,
  SysTick_IRQn               = -1,

  GPIO0_IRQn                 = 0,
  GPIO1_IRQn                 = 1,
  GPIO2_IRQn                 = 2,
  GPIO3_IRQn                 = 3,
  GPIO4_IRQn                 = 4,
  GPIO5_IRQn                 = 5,
  GPIO6_IRQn                 = 6,
  GPIO7_IRQn                 = 7,
  T16N0_IRQn                 = 8,
  T16N1_IRQn                 = 9,
  T16N2_IRQn                 = 10,
  T16N3_IRQn                 = 11,
  T32N0_IRQn                 = 12,
  T32N1_IRQn                 = 13,
  T32N2_IRQn                 = 14,
  T32N3_IRQn                 = 15,
  IWDT_IRQn                  = 16,
  WWDT_IRQn                  = 17,
  SPI2_IRQn                  = 18,
  SPI3_IRQn                  = 19,
  UART0_IRQn                 = 20,
  UART1_IRQn                 = 21,
  UART2_IRQn                 = 22,
  UART3_IRQn                 = 23,
  SPI0_IRQn                  = 24,
  SPI1_IRQn                  = 25,
  I2C0_IRQn                  = 26,
  I2C1_IRQn                  = 27,
  AES_IRQn                   = 28,
  ADC_IRQn                   = 29,
  Tsensor_IRQn               = 30,
  KINT_IRQn                  = 31,
  PLCRX_IRQn                 = 32,
  PLCTX_IRQn                 = 33,
  PLCTIMERINT_IRQn           = 34,
  Reserved1_IRQn             = 35,
  DMADONE_IRQn               = 36,
  DMAERR_IRQn                = 37,
  ECC_IRQn                   = 38,
  TRNG_IRQn                  = 39,
  Cache_IRQn                 = 40,
  USB_IRQn                   = 41,
  USB_BWERR_IRQn             = 42,
  ETHERNETINT_IRQn           = 43,
  LVD_IRQn                   = 44,
  COMP_IRQn                  = 45,

  IRQn_DEF
} IRQn_Type;

#include <stdint.h>
#include "core_cm3.h"


/****************************设备特殊寄存器结构定义*******************************/
#pragma anon_unions /* 允许匿名结构和匿名联合 */

/************************************* GPIO ***********************************/
typedef union
{
  struct
  {
    uint32_t PORT_0: 1;
    uint32_t PORT_1: 1;
    uint32_t PORT_2: 1;
    uint32_t PORT_3: 1;
    uint32_t PORT_4: 1;
    uint32_t PORT_5: 1;
    uint32_t PORT_6: 1;
    uint32_t PORT_7: 1;
    uint32_t PORT_8: 1;
    uint32_t PORT_9: 1;
    uint32_t PORT_10: 1;
    uint32_t PORT_11: 1;
    uint32_t PORT_12: 1;
    uint32_t PORT_13: 1;
    uint32_t PORT_14: 1;
    uint32_t PORT_15: 1;
    uint32_t PORT_16: 1;
    uint32_t PORT_17: 1;
    uint32_t PORT_18: 1;
    uint32_t PORT_19: 1;
    uint32_t PORT_20: 1;
    uint32_t PORT_21: 1;
    uint32_t PORT_22: 1;
    uint32_t PORT_23: 1;
    uint32_t PORT_24: 1;
    uint32_t PORT_25: 1;
    uint32_t PORT_26: 1;
    uint32_t PORT_27: 1;
    uint32_t PORT_28: 1;
    uint32_t PORT_29: 1;
    uint32_t PORT_30: 1;
    uint32_t PORT_31: 1;
  };
  uint32_t Word;
} GPIO_PORT_Typedef;

typedef union
{
  struct
  {
    uint32_t DATA_0: 1;
    uint32_t DATA_1: 1;
    uint32_t DATA_2: 1;
    uint32_t DATA_3: 1;
    uint32_t DATA_4: 1;
    uint32_t DATA_5: 1;
    uint32_t DATA_6: 1;
    uint32_t DATA_7: 1;
    uint32_t DATA_8: 1;
    uint32_t DATA_9: 1;
    uint32_t DATA_10: 1;
    uint32_t DATA_11: 1;
    uint32_t DATA_12: 1;
    uint32_t DATA_13: 1;
    uint32_t DATA_14: 1;
    uint32_t DATA_15: 1;
    uint32_t DATA_16: 1;
    uint32_t DATA_17: 1;
    uint32_t DATA_18: 1;
    uint32_t DATA_19: 1;
    uint32_t DATA_20: 1;
    uint32_t DATA_21: 1;
    uint32_t DATA_22: 1;
    uint32_t DATA_23: 1;
    uint32_t DATA_24: 1;
    uint32_t DATA_25: 1;
    uint32_t DATA_26: 1;
    uint32_t DATA_27: 1;
    uint32_t DATA_28: 1;
    uint32_t DATA_29: 1;
    uint32_t DATA_30: 1;
    uint32_t DATA_31: 1;
  };
  uint32_t Word;
} GPIO_DATA_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DATABSR_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DATABCR_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DATABRR_Typedef;

typedef union
{
  struct
  {
    uint32_t DIR_0: 1;
    uint32_t DIR_1: 1;
    uint32_t DIR_2: 1;
    uint32_t DIR_3: 1;
    uint32_t DIR_4: 1;
    uint32_t DIR_5: 1;
    uint32_t DIR_6: 1;
    uint32_t DIR_7: 1;
    uint32_t DIR_8: 1;
    uint32_t DIR_9: 1;
    uint32_t DIR_10: 1;
    uint32_t DIR_11: 1;
    uint32_t DIR_12: 1;
    uint32_t DIR_13: 1;
    uint32_t DIR_14: 1;
    uint32_t DIR_15: 1;
    uint32_t DIR_16: 1;
    uint32_t DIR_17: 1;
    uint32_t DIR_18: 1;
    uint32_t DIR_19: 1;
    uint32_t DIR_20: 1;
    uint32_t DIR_21: 1;
    uint32_t DIR_22: 1;
    uint32_t DIR_23: 1;
    uint32_t DIR_24: 1;
    uint32_t DIR_25: 1;
    uint32_t DIR_26: 1;
    uint32_t DIR_27: 1;
    uint32_t DIR_28: 1;
    uint32_t DIR_29: 1;
    uint32_t DIR_30: 1;
    uint32_t DIR_31: 1;
  };
  uint32_t Word;
} GPIO_DIR_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DIRBSR_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DIRBCR_Typedef;

typedef struct
{
  uint32_t Word;
} GPIO_DIRBRR_Typedef;

typedef union
{
  struct
  {
    uint32_t GPIO0FUNC: 2;
    uint32_t RESERVED0: 2;
    uint32_t GPIO1FUNC: 2;
    uint32_t RESERVED1: 2;
    uint32_t GPIO2FUNC: 2;
    uint32_t RESERVED2: 2;
    uint32_t GPIO3FUNC: 2;
    uint32_t RESERVED3: 2;
    uint32_t GPIO4FUNC: 2;
    uint32_t RESERVED4: 2;
    uint32_t GPIO5FUNC: 2;
    uint32_t RESERVED5: 2;
    uint32_t GPIO6FUNC: 2;
    uint32_t RESERVED6: 2;
    uint32_t GPIO7FUNC: 2;
    uint32_t RESERVED7: 2;
  };
  uint32_t Word;
} GPIO_FUNC0_Typedef;

typedef union
{
  struct
  {
    uint32_t GPIO8FUNC: 2;
    uint32_t RESERVED0: 2;
    uint32_t GPIO9FUNC: 2;
    uint32_t RESERVED1: 2;
    uint32_t GPIO10FUNC: 2;
    uint32_t RESERVED2: 2;
    uint32_t GPIO11FUNC: 2;
    uint32_t RESERVED3: 2;
    uint32_t GPIO12FUNC: 2;
    uint32_t RESERVED4: 2;
    uint32_t GPIO13FUNC: 2;
    uint32_t RESERVED5: 2;
    uint32_t GPIO14FUNC: 2;
    uint32_t RESERVED6: 2;
    uint32_t GPIO15FUNC: 2;
    uint32_t RESERVED7: 2;
  };
  uint32_t Word;
} GPIO_FUNC1_Typedef;

typedef union
{
  struct
  {
    uint32_t GPIO16FUNC: 2;
    uint32_t RESERVED0: 2;
    uint32_t GPIO17FUNC: 2;
    uint32_t RESERVED1: 2;
    uint32_t GPIO18FUNC: 2;
    uint32_t RESERVED2: 2;
    uint32_t GPIO19FUNC: 2;
    uint32_t RESERVED3: 2;
    uint32_t GPIO20FUNC: 2;
    uint32_t RESERVED4: 2;
    uint32_t GPIO21FUNC: 2;
    uint32_t RESERVED5: 2;
    uint32_t GPIO22FUNC: 2;
    uint32_t RESERVED6: 2;
    uint32_t GPIO23FUNC: 2;
    uint32_t RESERVED7: 2;
  };
  uint32_t Word;
} GPIO_FUNC2_Typedef;

typedef union
{
  struct
  {
    uint32_t GPIO24FUNC: 2;
    uint32_t RESERVED0: 2;
    uint32_t GPIO25FUNC: 2;
    uint32_t RESERVED1: 2;
    uint32_t GPIO26FUNC: 2;
    uint32_t RESERVED2: 2;
    uint32_t GPIO27FUNC: 2;
    uint32_t RESERVED3: 2;
    uint32_t GPIO28FUNC: 2;
    uint32_t RESERVED4: 2;
    uint32_t GPIO29FUNC: 2;
    uint32_t RESERVED5: 2;
    uint32_t GPIO30FUNC: 2;
    uint32_t RESERVED6: 2;
    uint32_t GPIO31FUNC: 2;
    uint32_t RESERVED7: 2;
  };
  uint32_t Word;
} GPIO_FUNC3_Typedef;

typedef union
{
  struct
  {
    uint32_t INEB_0: 1;
    uint32_t INEB_1: 1;
    uint32_t INEB_2: 1;
    uint32_t INEB_3: 1;
    uint32_t INEB_4: 1;
    uint32_t INEB_5: 1;
    uint32_t INEB_6: 1;
    uint32_t INEB_7: 1;
    uint32_t INEB_8: 1;
    uint32_t INEB_9: 1;
    uint32_t INEB_10: 1;
    uint32_t INEB_11: 1;
    uint32_t INEB_12: 1;
    uint32_t INEB_13: 1;
    uint32_t INEB_14: 1;
    uint32_t INEB_15: 1;
    uint32_t INEB_16: 1;
    uint32_t INEB_17: 1;
    uint32_t INEB_18: 1;
    uint32_t INEB_19: 1;
    uint32_t INEB_20: 1;
    uint32_t INEB_21: 1;
    uint32_t INEB_22: 1;
    uint32_t INEB_23: 1;
    uint32_t INEB_24: 1;
    uint32_t INEB_25: 1;
    uint32_t INEB_26: 1;
    uint32_t INEB_27: 1;
    uint32_t INEB_28: 1;
    uint32_t INEB_29: 1;
    uint32_t INEB_30: 1;
    uint32_t INEB_31: 1;
  };
  uint32_t Word;
} GPIO_INEB_Typedef;

typedef union
{
  struct
  {
    uint32_t ODE_0: 1;
    uint32_t ODE_1: 1;
    uint32_t ODE_2: 1;
    uint32_t ODE_3: 1;
    uint32_t ODE_4: 1;
    uint32_t ODE_5: 1;
    uint32_t ODE_6: 1;
    uint32_t ODE_7: 1;
    uint32_t ODE_8: 1;
    uint32_t ODE_9: 1;
    uint32_t ODE_10: 1;
    uint32_t ODE_11: 1;
    uint32_t ODE_12: 1;
    uint32_t ODE_13: 1;
    uint32_t ODE_14: 1;
    uint32_t ODE_15: 1;
    uint32_t ODE_16: 1;
    uint32_t ODE_17: 1;
    uint32_t ODE_18: 1;
    uint32_t ODE_19: 1;
    uint32_t ODE_20: 1;
    uint32_t ODE_21: 1;
    uint32_t ODE_22: 1;
    uint32_t ODE_23: 1;
    uint32_t ODE_24: 1;
    uint32_t ODE_25: 1;
    uint32_t ODE_26: 1;
    uint32_t ODE_27: 1;
    uint32_t ODE_28: 1;
    uint32_t ODE_29: 1;
    uint32_t ODE_30: 1;
    uint32_t ODE_31: 1;
  };
  uint32_t Word;
} GPIO_ODE_Typedef;

typedef union
{
  struct
  {
    uint32_t PUE_0: 1;
    uint32_t PUE_1: 1;
    uint32_t PUE_2: 1;
    uint32_t PUE_3: 1;
    uint32_t PUE_4: 1;
    uint32_t PUE_5: 1;
    uint32_t PUE_6: 1;
    uint32_t PUE_7: 1;
    uint32_t PUE_8: 1;
    uint32_t PUE_9: 1;
    uint32_t PUE_10: 1;
    uint32_t PUE_11: 1;
    uint32_t PUE_12: 1;
    uint32_t PUE_13: 1;
    uint32_t PUE_14: 1;
    uint32_t PUE_15: 1;
    uint32_t PUE_16: 1;
    uint32_t PUE_17: 1;
    uint32_t PUE_18: 1;
    uint32_t PUE_19: 1;
    uint32_t PUE_20: 1;
    uint32_t PUE_21: 1;
    uint32_t PUE_22: 1;
    uint32_t PUE_23: 1;
    uint32_t PUE_24: 1;
    uint32_t PUE_25: 1;
    uint32_t PUE_26: 1;
    uint32_t PUE_27: 1;
    uint32_t PUE_28: 1;
    uint32_t PUE_29: 1;
    uint32_t PUE_30: 1;
    uint32_t PUE_31: 1;
  };
  uint32_t Word;
} GPIO_PUE_Typedef;

typedef union
{
  struct
  {
    uint32_t PDE_0: 1;
    uint32_t PDE_1: 1;
    uint32_t PDE_2: 1;
    uint32_t PDE_3: 1;
    uint32_t PDE_4: 1;
    uint32_t PDE_5: 1;
    uint32_t PDE_6: 1;
    uint32_t PDE_7: 1;
    uint32_t PDE_8: 1;
    uint32_t PDE_9: 1;
    uint32_t PDE_10: 1;
    uint32_t PDE_11: 1;
    uint32_t PDE_12: 1;
    uint32_t PDE_13: 1;
    uint32_t PDE_14: 1;
    uint32_t PDE_15: 1;
    uint32_t PDE_16: 1;
    uint32_t PDE_17: 1;
    uint32_t PDE_18: 1;
    uint32_t PDE_19: 1;
    uint32_t PDE_20: 1;
    uint32_t PDE_21: 1;
    uint32_t PDE_22: 1;
    uint32_t PDE_23: 1;
    uint32_t PDE_24: 1;
    uint32_t PDE_25: 1;
    uint32_t PDE_26: 1;
    uint32_t PDE_27: 1;
    uint32_t PDE_28: 1;
    uint32_t PDE_29: 1;
    uint32_t PDE_30: 1;
    uint32_t PDE_31: 1;
  };
  uint32_t Word;
} GPIO_PDE_Typedef;

typedef union
{
  struct
  {
    uint32_t DS_0: 1;
    uint32_t DS_1: 1;
    uint32_t DS_2: 1;
    uint32_t DS_3: 1;
    uint32_t DS_4: 1;
    uint32_t DS_5: 1;
    uint32_t DS_6: 1;
    uint32_t DS_7: 1;
    uint32_t DS_8: 1;
    uint32_t DS_9: 1;
    uint32_t DS_10: 1;
    uint32_t DS_11: 1;
    uint32_t DS_12: 1;
    uint32_t DS_13: 1;
    uint32_t DS_14: 1;
    uint32_t DS_15: 1;
    uint32_t DS_16: 1;
    uint32_t DS_17: 1;
    uint32_t DS_18: 1;
    uint32_t DS_19: 1;
    uint32_t DS_20: 1;
    uint32_t DS_21: 1;
    uint32_t DS_22: 1;
    uint32_t DS_23: 1;
    uint32_t DS_24: 1;
    uint32_t DS_25: 1;
    uint32_t DS_26: 1;
    uint32_t DS_27: 1;
    uint32_t DS_28: 1;
    uint32_t DS_29: 1;
    uint32_t DS_30: 1;
    uint32_t DS_31: 1;
  };
  uint32_t Word;
} GPIO_DS_Typedef;

typedef union
{
  struct
  {
    uint32_t PINTIE_V: 8;
    uint32_t PMASK_V: 8;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} GPIO_PINTIE_Typedef;

typedef union
{
  struct
  {
    uint32_t PINTIF_0: 1;
    uint32_t PINTIF_1: 1;
    uint32_t PINTIF_2: 1;
    uint32_t PINTIF_3: 1;
    uint32_t PINTIF_4: 1;
    uint32_t PINTIF_5: 1;
    uint32_t PINTIF_6: 1;
    uint32_t PINTIF_7: 1;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} GPIO_PINTIF_Typedef;

typedef union
{
  struct
  {
    uint32_t PINT0_SEL: 3;
    uint32_t RESERVED0: 1;
    uint32_t PINT1_SEL: 3;
    uint32_t RESERVED1: 1;
    uint32_t PINT2_SEL: 3;
    uint32_t RESERVED2: 1;
    uint32_t PINT3_SEL: 3;
    uint32_t RESERVED3: 1;
    uint32_t PINT4_SEL: 3;
    uint32_t RESERVED4: 1;
    uint32_t PINT5_SEL: 3;
    uint32_t RESERVED5: 1;
    uint32_t PINT6_SEL: 3;
    uint32_t RESERVED6: 1;
    uint32_t PINT7_SEL: 3;
    uint32_t RESERVED7: 1;
  };
  uint32_t Word;
} GPIO_PINTSEL_Typedef;

typedef union
{
  struct
  {
    uint32_t PINT0_CFG: 3;
    uint32_t RESERVED0: 1;
    uint32_t PINT1_CFG: 3;
    uint32_t RESERVED1: 1;
    uint32_t PINT2_CFG: 3;
    uint32_t RESERVED2: 1;
    uint32_t PINT3_CFG: 3;
    uint32_t RESERVED3: 1;
    uint32_t PINT4_CFG: 3;
    uint32_t RESERVED4: 1;
    uint32_t PINT5_CFG: 3;
    uint32_t RESERVED5: 1;
    uint32_t PINT6_CFG: 3;
    uint32_t RESERVED6: 1;
    uint32_t PINT7_CFG: 3;
    uint32_t RESERVED7: 1;
  };
  uint32_t Word;
} GPIO_PINTCFG_Typedef;

typedef union
{
  struct
  {
    uint32_t KINTIE: 8;
    uint32_t KMASK: 8;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} GPIO_KINTIE_Typedef;

typedef union
{
  struct
  {
    uint32_t KINTIF_0: 1;
    uint32_t KINTIF_1: 1;
    uint32_t KINTIF_2: 1;
    uint32_t KINTIF_3: 1;
    uint32_t KINTIF_4: 1;
    uint32_t KINTIF_5: 1;
    uint32_t KINTIF_6: 1;
    uint32_t KINTIF_7: 1;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} GPIO_KINTIF_Typedef;

typedef union
{
  struct
  {
    uint32_t KINT0_SEL: 3;
    uint32_t RESERVED0: 1;
    uint32_t KINT1_SEL: 3;
    uint32_t RESERVED1: 1;
    uint32_t KINT2_SEL: 3;
    uint32_t RESERVED2: 1;
    uint32_t KINT3_SEL: 3;
    uint32_t RESERVED3: 1;
    uint32_t KINT4_SEL: 3;
    uint32_t RESERVED4: 1;
    uint32_t KINT5_SEL: 3;
    uint32_t RESERVED5: 1;
    uint32_t KINT6_SEL: 3;
    uint32_t RESERVED6: 1;
    uint32_t KINT7_SEL: 3;
    uint32_t RESERVED7: 1;
  };
  uint32_t Word;
} GPIO_KINTSEL_Typedef;

typedef union
{
  struct
  {
    uint32_t KINT0_CFG: 3;
    uint32_t RESERVED0: 1;
    uint32_t KINT1_CFG: 3;
    uint32_t RESERVED1: 1;
    uint32_t KINT2_CFG: 3;
    uint32_t RESERVED2: 1;
    uint32_t KINT3_CFG: 3;
    uint32_t RESERVED3: 1;
    uint32_t KINT4_CFG: 3;
    uint32_t RESERVED4: 1;
    uint32_t KINT5_CFG: 3;
    uint32_t RESERVED5: 1;
    uint32_t KINT6_CFG: 3;
    uint32_t RESERVED6: 1;
    uint32_t KINT7_CFG: 3;
    uint32_t RESERVED7: 1;
  };
  uint32_t Word;
} GPIO_KINTCFG_Typedef;

typedef union
{
  uint32_t Word;
} GPIO_IOFLTSEL_Typedef;

typedef union
{
  struct
  {
    uint32_t TX0PS: 2;
    uint32_t TX0PLV: 1;
    uint32_t RESERVED0: 1;
    uint32_t TX0_S0: 1;
    uint32_t TX0_S1: 1;
    uint32_t TX0_S2: 1;
    uint32_t TX0_S3: 1;
    uint32_t TX1PS: 2;
    uint32_t TX1PLV: 1;
    uint32_t RESERVED1: 1;
    uint32_t TX1_S0: 1;
    uint32_t TX1_S1: 1;
    uint32_t TX1_S2: 1;
    uint32_t TX1_S3: 1;
    uint32_t RESERVED2: 16;
  };
  uint32_t Word;
} GPIO_TXPWM_Typedef;

typedef union
{
  struct
  {
    uint32_t BUZEN: 1;
    uint32_t RESERVED0: 7;
    uint32_t BUZ_LOAD: 20;
    uint32_t RESERVED1: 4;
  };
  uint32_t Word;
} GPIO_BUZC_Typedef;

typedef struct
{
  __I GPIO_PORT_Typedef PORT;
  __IO GPIO_DATA_Typedef DATA;
  __O GPIO_DATABSR_Typedef DATABSR;
  __O GPIO_DATABCR_Typedef DATABCR;
  __O GPIO_DATABRR_Typedef DATABRR;
  __IO GPIO_DIR_Typedef DIR;
  __O GPIO_DIRBSR_Typedef DIRBSR;
  __O GPIO_DIRBCR_Typedef DIRBCR;
  __O GPIO_DIRBRR_Typedef DIRBRR;
  __IO GPIO_FUNC0_Typedef FUNC0;
  __IO GPIO_FUNC1_Typedef FUNC1;
  __IO GPIO_FUNC2_Typedef FUNC2;
  __IO GPIO_FUNC3_Typedef FUNC3;
  __IO GPIO_INEB_Typedef INEB;
  __IO GPIO_ODE_Typedef ODE;
  __IO GPIO_PUE_Typedef PUE;
  __IO GPIO_PDE_Typedef PDE;
  __IO GPIO_DS_Typedef PDS;
  uint32_t RESERVED0[174];
  __IO GPIO_PINTIE_Typedef PINTIE;
  __IO GPIO_PINTIF_Typedef PINTIF;
  __IO GPIO_PINTSEL_Typedef PINTSEL;
  __IO GPIO_PINTCFG_Typedef PINTCFG;
  __IO GPIO_KINTIE_Typedef KINTIE;
  __IO GPIO_KINTIF_Typedef KINTIF;
  __IO GPIO_KINTSEL_Typedef KINTSEL;
  __IO GPIO_KINTCFG_Typedef KINTCFG;
  uint32_t RESERVED1[4];
  __IO GPIO_IOFLTSEL_Typedef IOFLTSEL;
  uint32_t RESERVED2[19];
  __IO GPIO_TXPWM_Typedef TXPWM;
  __IO GPIO_BUZC_Typedef BUZC;
} GPIO_TypeDef;

/************************************* T16N ***********************************/
typedef union
{
  struct
  {
    uint32_t CNT0: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_CNT0_Typedef;

typedef union
{
  struct
  {
    uint32_t CNT1: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_CNT1_Typedef;

typedef union
{
  struct
  {
    uint32_t PRECNT: 8;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} T16N_PRECNT_Typedef;

typedef union
{
  struct
  {
    uint32_t PREMAT: 8;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} T16N_PREMAT_Typedef;

typedef union
{
  struct
  {
    uint32_t EN: 1;
    uint32_t CS: 2;
    uint32_t SYNC: 1;
    uint32_t EDGE: 2;
    uint32_t MOD: 2;
    uint32_t MAT0S: 2;
    uint32_t MAT1S: 2;
    uint32_t MAT2S: 2;
    uint32_t MAT3S: 2;
    uint32_t ASYWEN: 1;
    uint32_t RESERVED0: 15;
  };
  uint32_t Word;
} T16N_CON0_Typedef;

typedef union
{
  struct
  {
    uint32_t CAPPE: 1;
    uint32_t CAPNE: 1;
    uint32_t CAPIS0: 1;
    uint32_t CAPIS1: 1;
    uint32_t CAPT: 4;
    uint32_t CAPL0: 1;
    uint32_t CAPL1: 1;
    uint32_t RESERVED0: 22;
  };
  uint32_t Word;
} T16N_CON1_Typedef;

typedef union
{
  struct
  {
    uint32_t MOE0: 1;
    uint32_t MOE1: 1;
    uint32_t POL0: 1;
    uint32_t POL1: 1;
    uint32_t PWMMOD: 2;
    uint32_t PWMDZE: 1;
    uint32_t RESERVED0: 1;
    uint32_t MOM0: 2;
    uint32_t MOM1: 2;
    uint32_t MOM2: 2;
    uint32_t MOM3: 2;
    uint32_t PWMBKE0: 1;
    uint32_t PWMBKE1: 1;
    uint32_t PWMBKL0: 1;
    uint32_t PWMBKL1: 1;
    uint32_t RESERVED1: 2;
    uint32_t PWMBKP0: 1;
    uint32_t PWMBKP1: 1;
    uint32_t PWMBKF: 1;
    uint32_t RESERVED2: 7;
  };
  uint32_t Word;
} T16N_CON2_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0IE: 1;
    uint32_t MAT1IE: 1;
    uint32_t MAT2IE: 1;
    uint32_t MAT3IE: 1;
    uint32_t TOP0IE: 1;
    uint32_t TOP1IE: 1;
    uint32_t CAP0IE: 1;
    uint32_t CAP1IE: 1;
    uint32_t PBK0IE: 1;
    uint32_t PBK1IE: 1;
    uint32_t RESERVED0: 22;
  };
  uint32_t Word;
} T16N_IE_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0IF: 1;
    uint32_t MAT1IF: 1;
    uint32_t MAT2IF: 1;
    uint32_t MAT3IF: 1;
    uint32_t TOP0IF: 1;
    uint32_t TOP1IF: 1;
    uint32_t CAP0IF: 1;
    uint32_t CAP1IF: 1;
    uint32_t PBK0IF: 1;
    uint32_t PBK1IF: 1;
    uint32_t RESERVED0: 22;
  };
  uint32_t Word;
} T16N_IF_Typedef;

typedef union
{
  struct
  {
    uint32_t PDZ: 8;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} T16N_PDZ_Typedef;

typedef union
{
  struct
  {
    uint32_t RESERVED0: 1;
    uint32_t P0MAT0TRE: 1;
    uint32_t P0MAT1TRE: 1;
    uint32_t P0TOP0TRE: 1;
    uint32_t RESERVED1: 1;
    uint32_t P1MAT2TRE: 1;
    uint32_t P1MAT3TRE: 1;
    uint32_t P1TOP1TRE: 1;
    uint32_t RESERVED2: 24;
  };
  uint32_t Word;
} T16N_PTR_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_MAT0_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT1: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_MAT1_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT2: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_MAT2_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT3: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_MAT3_Typedef;

typedef union
{
  struct
  {
    uint32_t TOP0: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_TOP0_Typedef;

typedef union
{
  struct
  {
    uint32_t TOP1: 16;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} T16N_TOP1_Typedef;

typedef struct
{
  __IO T16N_CNT0_Typedef CNT0;
  __IO T16N_CNT1_Typedef CNT1;
  __IO T16N_PRECNT_Typedef PRECNT;
  __IO T16N_PREMAT_Typedef PREMAT;
  __IO T16N_CON0_Typedef CON0;
  __IO T16N_CON1_Typedef CON1;
  __IO T16N_CON2_Typedef CON2;
  uint32_t RESERVED0;
  __IO T16N_IE_Typedef IE;
  __IO T16N_IF_Typedef IF;
  __IO T16N_PDZ_Typedef PDZ;
  __IO T16N_PTR_Typedef PTR;
  __IO T16N_MAT0_Typedef MAT0;
  __IO T16N_MAT1_Typedef MAT1;
  __IO T16N_MAT2_Typedef MAT2;
  __IO T16N_MAT3_Typedef MAT3;
  __IO T16N_TOP0_Typedef TOP0;
  __IO T16N_TOP1_Typedef TOP1;
} T16N_TypeDef;

/************************************* T32N ***********************************/
typedef union
{
  struct
  {
    uint32_t CNT0: 32;
  };
  uint32_t Word;
} T32N_CNT_Typedef;

typedef union
{
  struct
  {
    uint32_t EN: 1;
    uint32_t CS: 2;
    uint32_t SYNC: 1;
    uint32_t EDGE: 2;
    uint32_t MOD: 2;
    uint32_t MAT0S: 2;
    uint32_t MAT1S: 2;
    uint32_t MAT2S: 2;
    uint32_t MAT3S: 2;
    uint32_t ASYNC_WREN: 1;
    uint32_t RESERVED0: 15;
  };
  uint32_t Word;
} T32N_CON0_Typedef;

typedef union
{
  struct
  {
    uint32_t CAPPE: 1;
    uint32_t CAPNE: 1;
    uint32_t CAPIS0: 1;
    uint32_t CAPIS1: 1;
    uint32_t CAPT: 4;
    uint32_t CAPL0: 1;
    uint32_t CAPL1: 1;
    uint32_t RESERVED0: 6;
    uint32_t MOE0: 1;
    uint32_t MOE1: 1;
    uint32_t RESERVED1: 6;
    uint32_t MOM0: 2;
    uint32_t MOM1: 2;
    uint32_t MOM2: 2;
    uint32_t MOM3: 2;
  };
  uint32_t Word;
} T32N_CON1_Typedef;

typedef union
{
  struct
  {
    uint32_t PRECNT: 8;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} T32N_PRECNT_Typedef;

typedef union
{
  struct
  {
    uint32_t PREMAT: 8;
    uint32_t RESERVED0: 24;
  };
  uint32_t Word;
} T32N_PREMAT_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0IE: 1;
    uint32_t MAT1IE: 1;
    uint32_t MAT2IE: 1;
    uint32_t MAT3IE: 1;
    uint32_t IE: 1;
    uint32_t CAP0IE: 1;
    uint32_t CAP1IE: 1;
    uint32_t RESERVED0: 25;
  };
  uint32_t Word;
} T32N_IE_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0IF: 1;
    uint32_t MAT1IF: 1;
    uint32_t MAT2IF: 1;
    uint32_t MAT3IF: 1;
    uint32_t IF: 1;
    uint32_t CAP0IF: 1;
    uint32_t CAP1IF: 1;
    uint32_t RESERVED0: 25;
  };
  uint32_t Word;
} T32N_IF_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT0: 32;
  };
  uint32_t Word;
} T32N_MAT0_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT1: 32;
  };
  uint32_t Word;
} T32N_MAT1_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT2: 32;
  };
  uint32_t Word;
} T32N_MAT2_Typedef;

typedef union
{
  struct
  {
    uint32_t MAT3: 32;
  };
  uint32_t Word;
} T32N_MAT3_Typedef;

typedef struct
{
  __IO T32N_CNT_Typedef CNT;
  __IO T32N_CON0_Typedef CON0;
  __IO T32N_CON1_Typedef CON1;
  uint32_t RESERVED0;
  __IO T32N_PRECNT_Typedef PRECNT;
  __IO T32N_PREMAT_Typedef PREMAT;
  __IO T32N_IE_Typedef IE;
  __IO T32N_IF_Typedef IF;
  __IO T32N_MAT0_Typedef MAT0;
  __IO T32N_MAT1_Typedef MAT1;
  __IO T32N_MAT2_Typedef MAT2;
  __IO T32N_MAT3_Typedef MAT3;
} T32N_TypeDef;

/************************************* UART ***********************************/
typedef union
{
  struct
  {
    uint32_t TXEN: 1;
    uint32_t TRST: 1;
    uint32_t TBCLR: 1;
    uint32_t TXI: 1;
    uint32_t RESERVED0: 4;
    uint32_t TXMOD: 4;
    uint32_t TXP: 1;
    uint32_t TXFS: 1;
    uint32_t RESERVED1: 2;
    uint32_t RXEN: 1;
    uint32_t RRST: 1;
    uint32_t RBCLR: 1;
    uint32_t RXI: 1;
    uint32_t BDEN: 1;
    uint32_t IDEN: 1;
    uint32_t RESERVED2: 2;
    uint32_t RXMOD: 4;
    uint32_t RXP: 1;
    uint32_t RESERVED3: 3;
  };
  uint32_t Word;
} UART_CON0_Typedef;

typedef union
{
  struct
  {
    uint32_t TBIM: 2;
    uint32_t RESERVED0: 2;
    uint32_t RBIM: 2;
    uint32_t RESERVED1: 2;
    uint32_t BCS: 3;
    uint32_t RESERVED2: 1;
    uint32_t BDM: 2;
    uint32_t RESERVED3: 2;
    uint32_t IDM: 2;
    uint32_t RESERVED4: 14;
  };
  uint32_t Word;
} UART_CON1_Typedef;

typedef union
{
  struct
  {
    uint32_t TXDMAEN: 1;
    uint32_t RXDMAEN: 1;
    uint32_t DMAONERR: 1;
    uint32_t RESERVED0: 29;
  };
  uint32_t Word;
} UART_DMACR_Typedef;

typedef union
{
  struct
  {
    uint32_t BRFRA: 4;
    uint32_t BRINT: 12;
    uint32_t RESERVED0: 16;
  };
  uint32_t Word;
} UART_BRR_Typedef;

typedef union
{
  struct
  {
    uint32_t TBPTR: 4;
    uint32_t TBOV: 1;
    uint32_t TXBUSY: 1;
    uint32_t RESERVED0: 2;
    uint32_t RBPTR: 4;
    uint32_t RBOV: 1;
    uint32_t RXBUSY: 1;
    uint32_t RESERVED1: 2;
    uint32_t FER0: 1;
    uint32_t PER0: 1;
    uint32_t FER1: 1;
    uint32_t PER1: 1;
    uint32_t FER2: 1;
    uint32_t PER2: 1;
    uint32_t FER3: 1;
    uint32_t PER3: 1;
    uint32_t RESERVED2: 8;
  };
  uint32_t Word;
} UART_STA_Typedef;

typedef union
{
  struct
  {
    uint32_t TBIE: 1;
    uint32_t TCIE: 1;
    uint32_t RESERVED0: 6;
    uint32_t TBWEIE: 1;
    uint32_t TBWOIE: 1;
    uint32_t RESERVED1: 6;
    uint32_t RBIE: 1;
    uint32_t IDIE: 1;
    uint32_t RESERVED2: 6;
    uint32_t ROIE: 1;
    uint32_t FEIE: 1;
    uint32_t PEIE: 1;
    uint32_t BDEIE: 1;
    uint32_t RBREIE: 1;
    uint32_t RBROIE: 1;
    uint32_t RESERVED3: 2;
  };
  uint32_t Word;
} UART_IE_Typedef;

typedef union
{
  struct
  {
    uint32_t TBIF: 1;
    uint32_t TCIF: 1;
    uint32_t RESERVED0: 6;
    uint32_t TBWEIF: 1;
    uint32_t TBWOIF: 1;
    uint32_t RESERVED1: 6;
    uint32_t RBIF: 1;
    uint32_t IDIF: 1;
    uint32_t RESERVED2: 6;
    uint32_t ROIF: 1;
    uint32_t FEIF: 1;
    uint32_t PEIF: 1;
    uint32_t BDEIF: 1;
    uint32_t RBREIF: 1;
    uint32_t RBROIF: 1;
    uint32_t RESERVED3: 2;
  };
  uint32_t Word;
} UART_IF_Typedef;

typedef union
{
  uint8_t Byte[4];
  uint16_t HalfWord[2];
  uint32_t Word;
} UART_TBW_Typedef;

typedef union
{
  uint8_t Byte[4];
  uint16_t HalfWord[2];
  uint32_t Word;
} UART_RBR_Typedef;

typedef union
{
  struct
  {
    uint32_t TB0: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP0: 1;
    uint32_t TBFF0: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB0_Typedef;

typedef union
{
  struct
  {
    uint32_t TB1: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP1: 1;
    uint32_t TBFF1: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB1_Typedef;

typedef union
{
  struct
  {
    uint32_t TB2: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP2: 1;
    uint32_t TBFF2: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB2_Typedef;

typedef union
{
  struct
  {
    uint32_t TB3: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP3: 1;
    uint32_t TBFF3: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB3_Typedef;

typedef union
{
  struct
  {
    uint32_t TB4: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP4: 1;
    uint32_t TBFF4: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB4_Typedef;

typedef union
{
  struct
  {
    uint32_t TB5: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP5: 1;
    uint32_t TBFF5: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB5_Typedef;

typedef union
{
  struct
  {
    uint32_t TB6: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP6: 1;
    uint32_t TBFF6: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB6_Typedef;

typedef union
{
  struct
  {
    uint32_t TB7: 9;
    uint32_t RESERVED0: 3;
    uint32_t TP7: 1;
    uint32_t TBFF7: 1;
    uint32_t RESERVED1: 18;
  };
  uint32_t Word;
} UART_TB7_Typedef;

typedef union
{
  struct
  {
    uint32_t RB0: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP0: 1;
    uint32_t RBFF0: 1;
    uint32_t FE0: 1;
    uint32_t PE0: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB0_Typedef;

typedef union
{
  struct
  {
    uint32_t RB1: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP1: 1;
    uint32_t RBFF1: 1;
    uint32_t FE1: 1;
    uint32_t PE1: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB1_Typedef;

typedef union
{
  struct
  {
    uint32_t RB2: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP2: 1;
    uint32_t RBFF2: 1;
    uint32_t FE2: 1;
    uint32_t PE2: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB2_Typedef;

typedef union
{
  struct
  {
    uint32_t RB3: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP3: 1;
    uint32_t RBFF3: 1;
    uint32_t FE3: 1;
    uint32_t PE3: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB3_Typedef;

typedef union
{
  struct
  {
    uint32_t RB4: 5;
    uint32_t RESERVED0: 3;
    uint32_t RP4: 1;
    uint32_t RBFF4: 1;
    uint32_t FE4: 1;
    uint32_t PE4: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB4_Typedef;

typedef union
{
  struct
  {
    uint32_t RB5: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP5: 1;
    uint32_t RBFF5: 1;
    uint32_t FE5: 1;
    uint32_t PE5: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB5_Typedef;

typedef union
{
  struct
  {
    uint32_t RB6: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP6: 1;
    uint32_t RBFF6: 1;
    uint32_t FE6: 1;
    uint32_t PE6: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB6_Typedef;

typedef union
{
  struct
  {
    uint32_t RB7: 9;
    uint32_t RESERVED0: 3;
    uint32_t RP7: 1;
    uint32_t RBFF7: 1;
    uint32_t FE7: 1;
    uint32_t PE7: 1;
    uint32_t RESERVED1: 16;
  };
  uint32_t Word;
} UART_RB7_Typedef;

typedef struct
{
  __IO UART_CON0_Typedef CON0;
  __IO UART_CON1_Typedef CON1;
  __IO UART_DMACR_Typedef DMACR;
  uint32_t RESERVED0[1];
  __IO UART_BRR_Typedef BRR;
  __I UART_STA_Typedef STA;
  __IO UART_IE_Typedef IE;
  __IO UART_IF_Typedef IF;
  __O UART_TBW_Typedef TBW;
  __I UART_RBR_Typedef RBR;
  uint32_t RESERVED1[6];
  __I UART_TB0_Typedef TB0;
  __I UART_TB1_Typedef TB1;
  __I UART_TB2_Typedef TB2;
  __I UART_TB3_Typedef TB3;
  __I UART_TB4_Typedef TB4;
  __I UART_TB5_Typedef TB5;
  __I UART_TB6_Typedef TB6;
  __I UART_TB7_Typedef TB7;
  __I UART_RB0_Typedef RB0;
  __I UART_RB1_Typedef RB1;
  __I UART_RB2_Typedef RB2;
  __I UART_RB3_Typedef RB3;
  __I UART_RB4_Typedef RB4;
  __I UART_RB5_Typedef RB5;
  __I UART_RB6_Typedef RB6;
  __I UART_RB7_Typedef RB7;
} UART_TypeDef;


/**********************************物理地址映射*********************************/
/* Base addresses */
#define FLASH_BASE (0x00000000UL)
#define SRAM_BASE (0x20000000UL)
#define APB_BASE (0x40000000UL)

/* APB peripherals */
#define GPIO_BASE (APB_BASE + 0x00400)
#define T16N0_BASE (APB_BASE + 0x02000)
#define T16N1_BASE (APB_BASE + 0x02400)
#define T16N2_BASE (APB_BASE + 0x02800)
#define T16N3_BASE (APB_BASE + 0x02C00)
#define T32N0_BASE (APB_BASE + 0x04000)
#define T32N1_BASE (APB_BASE + 0x04400)
#define T32N2_BASE (APB_BASE + 0x04800)
#define T32N3_BASE (APB_BASE + 0x04C00)
#define UART0_BASE (APB_BASE + 0x06000)

/***************************************外设定义************************/
#define GPIO  ((GPIO_TypeDef *) GPIO_BASE)
#define T16N0 ((T16N_TypeDef *) T16N0_BASE)
#define T16N1 ((T16N_TypeDef *) T16N1_BASE)
#define T16N2 ((T16N_TypeDef *) T16N2_BASE)
#define T16N3 ((T16N_TypeDef *) T16N3_BASE)
#define T32N0 ((T32N_TypeDef *) T32N0_BASE)
#define T32N1 ((T32N_TypeDef *) T32N1_BASE)
#define T32N2 ((T32N_TypeDef *) T32N2_BASE)
#define T32N3 ((T32N_TypeDef *) T32N3_BASE)
#define UART0 ((UART_TypeDef *) UART0_BASE)

#define  CPU_CLK                 (150000000)  /* 150MHz */
#define  PCLK                    (75000000)   /* 75MHz */
typedef unsigned int OS_CPU_SR;

static inline unsigned int OS_ENTER_CRITICAL()
{
    register uint32_t __regPriMask         __ASM("primask");
    uint32_t ret = __regPriMask;
    __regPriMask = 0x01;
    return(ret);
}  

#define OS_EXIT_CRITICAL(cpu_sr)    __set_PRIMASK(cpu_sr)

#define _OS_ENTER_CRITICAL()                         \
    uint32_t cpu_sr = __get_PRIMASK();               \
    __disable_irq();

#define _OS_EXIT_CRITICAL()         __set_PRIMASK(cpu_sr)

#define OS_CRITICAL_PARA     uint32_t cpu_sr;
#define __OS_ENTER_CRITICAL()                         \
    cpu_sr = __get_PRIMASK();               \
    __disable_irq();

#define __OS_EXIT_CRITICAL()         __set_PRIMASK(cpu_sr)
#endif /* __SSC1667_H */


