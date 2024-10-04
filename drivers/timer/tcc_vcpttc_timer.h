/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TCC_VCPTTC_TIMER_H
#define TCC_VCPTTC_TIMER_H

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <soc.h>
#include <zephyr/drivers/timer/system_timer.h>
#include "xlnx_psttc_timer_priv.h"

#include <string.h>

typedef unsigned char boolean; /* for use with TRUE/FALSE        */

#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#define MCU_BSP_CKC_BASE    (0xA0F24000UL)
#define MCU_BSP_SUBSYS_BASE (0xA0F20000UL)

#define CLOCK_BASE_ADDR (MCU_BSP_CKC_BASE)

#define CLOCK_XIN_CLK_RATE (12000000UL) // 12MHz

#define CLOCK_PLL_MAX_NUM (2)
#define CLOCK_SRC_MAX_NUM (6) //  ((CLOCK_PLL_MAX_NUM * 2 ) + 2)

/*
 * Error Codes
 */
typedef enum SALErrorCode {
	SAL_ERR_INIT = 100,              /**< Initialization error for each APIs             */
	SAL_ERR_NO_SPACE = 101,          /**< No more space (memory, channel, etc)           */
	SAL_ERR_INVALID_PARAMETER = 102, /**< Invalid parameter is passed                    */
	SAL_ERR_NOT_SUPPORT = 103,       /**< Not supported operation or resources           */
	SAL_ERR_TIMEOUT = 104,           /**< Timed out while processing                     */
	SAL_ERR_INVALID_HANDLE = 105,    /**< Invalid handle is detected                     */
	SAL_ERR_NO_DATA = 106,           /**< No data                                        */
	SAL_ERR_UNDEF_STATE = 107,       /**< Not defined state                              */
	SAL_ERR_FAIL_CREATE = 108,       /**< Fail to create a component(Task, event, etc)   */
	SAL_ERR_FAIL_GET_DATA = 109,     /**< Fail to get data from a component              */
	SAL_ERR_FAIL_SEND_DATA = 110,    /**< Fail to send data to a component               */
	SAL_ERR_FAIL_START = 111,        /**< Fail to start a component                      */
	SAL_ERR_FAIL_DELETE = 112,       /**< Fail to delete a job of a component            */
	SAL_ERR_FAIL_RELEASE = 113,      /**< Fail to release a job of a component           */
	SAL_ERR_UNINIT_ITEM = 114,       /**< Uninitialized item, variable, or contents      */
	SAL_ERR_OUTOF_SIZE = 115,        /**< Size overflow or is lower than threshold       */
	SAL_ERR_FAIL_GET_KEY = 116,      /**< Fail to get key or ownership                   */
	SAL_ERR_FAIL_SET_CONFIG = 117,   /**< Fail to set configuration or status            */
	SAL_ERR_NOT_USEFUL = 118         /**< The status is not available                    */

} SALErrorCode_t;

typedef enum CLOCK_PCLK_CTRL_SEL {
	CLOCK_PCLKCTRL_SEL_PLL0 = 0,
	CLOCK_PCLKCTRL_SEL_PLL1 = 1,
	CLOCK_PCLKCTRL_SEL_XIN = 5,
	CLOCK_PCLKCTRL_SEL_PLL0DIV = 10,
	CLOCK_PCLKCTRL_SEL_PLL1DIV = 11,
	CLOCK_PCLKCTRL_SEL_XINDIV = 23
} CLOCKPclkctrlSel_t;

/* I/O Bus pwdn/swreset */
typedef enum CLOCK_IO_BUS {
	CLOCK_IOBUS_SFMC = 0,
	CLOCK_IOBUS_IMC = 1,
	CLOCK_IOBUS_PFLASH = 2,
	CLOCK_IOBUS_DFLASH = 3,
	CLOCK_IOBUS_GIC = 4,
	CLOCK_IOBUS_SOC400 = 5,
	CLOCK_IOBUS_DMA_CON0 = 6,
	CLOCK_IOBUS_DMA_CON1 = 7,
	CLOCK_IOBUS_DMA_CON2 = 8,
	CLOCK_IOBUS_DMA_CON3 = 9,
	CLOCK_IOBUS_DMA_CON4 = 10,
	CLOCK_IOBUS_DMA_CON5 = 11,
	CLOCK_IOBUS_DMA_CON6 = 12,
	CLOCK_IOBUS_DMA_CON7 = 13,
	CLOCK_IOBUS_CAN0 = 14,
	CLOCK_IOBUS_CAN1 = 15,
	CLOCK_IOBUS_CAN2 = 16,
	CLOCK_IOBUS_CAN_CONF = 17,
	CLOCK_IOBUS_UART0 = 18,
	CLOCK_IOBUS_UART1 = 19,
	CLOCK_IOBUS_UART2 = 20,
	CLOCK_IOBUS_UART3 = 21,
	CLOCK_IOBUS_UART4 = 22,
	CLOCK_IOBUS_UART5 = 23,
	CLOCK_IOBUS_CONF = 24,
	CLOCK_IOBUS_I2C0 = 25,
	CLOCK_IOBUS_I2C1 = 26,
	CLOCK_IOBUS_I2C2 = 27,
	CLOCK_IOBUS_I2C3 = 28,
	CLOCK_IOBUS_I2C4 = 29,
	CLOCK_IOBUS_I2C5 = 30,
	CLOCK_IOBUS_I2C_M_PORTCFG = 31,
	CLOCK_IOBUS_PWM0 = 32,      /* HCLK_MASK1[0] */
	CLOCK_IOBUS_PWM1 = 33,      /* HCLK_MASK1[1] */
	CLOCK_IOBUS_PWM2 = 34,      /* HCLK_MASK1[2] */
	CLOCK_IOBUS_PWM_CONF = 35,  /* HCLK_MASK1[3] */
	CLOCK_IOBUS_ICTC0 = 36,     /* HCLK_MASK1[4] */
	CLOCK_IOBUS_ICTC1 = 37,     /* HCLK_MASK1[5] */
	CLOCK_IOBUS_ICTC2 = 38,     /* HCLK_MASK1[6] */
	CLOCK_IOBUS_ICTC3 = 39,     /* HCLK_MASK1[7] */
	CLOCK_IOBUS_ICTC4 = 40,     /* HCLK_MASK1[8] */
	CLOCK_IOBUS_ICTC5 = 41,     /* HCLK_MASK1[9] */
	CLOCK_IOBUS_ADC0 = 42,      /* HCLK_MASK1[10] */
	CLOCK_IOBUS_ADC1 = 43,      /* HCLK_MASK1[11] */
	CLOCK_IOBUS_TIMER0 = 44,    /* HCLK_MASK1[12] */
	CLOCK_IOBUS_TIMER1 = 45,    /* HCLK_MASK1[13] */
	CLOCK_IOBUS_TIMER2 = 46,    /* HCLK_MASK1[14] */
	CLOCK_IOBUS_TIMER3 = 47,    /* HCLK_MASK1[15] */
	CLOCK_IOBUS_TIMER4 = 48,    /* HCLK_MASK1[16] */
	CLOCK_IOBUS_TIMER5 = 49,    /* HCLK_MASK1[17] */
	CLOCK_IOBUS_TIMER6 = 50,    /* HCLK_MASK1[18] */
	CLOCK_IOBUS_TIMER7 = 51,    /* HCLK_MASK1[19] */
	CLOCK_IOBUS_TIMER8 = 52,    /* HCLK_MASK1[20] */
	CLOCK_IOBUS_TIMER9 = 53,    /* HCLK_MASK1[21] */
	CLOCK_IOBUS_GPSB0 = 54,     /* HCLK_MASK1[22] */
	CLOCK_IOBUS_GPSB1 = 55,     /* HCLK_MASK1[23] */
	CLOCK_IOBUS_GPSB2 = 56,     /* HCLK_MASK1[24] */
	CLOCK_IOBUS_GPSB3 = 57,     /* HCLK_MASK1[25] */
	CLOCK_IOBUS_GPSB4 = 58,     /* HCLK_MASK1[26] */
	CLOCK_IOBUS_GPSB_CONF = 59, /* HCLK_MASK1[27] */
	CLOCK_IOBUS_GPSB_SM = 60,   /* HCLK_MASK1[28] */
	CLOCK_IOBUS_I2S = 61,       /* HCLK_MASK1[29] */
	CLOCK_IOBUS_GMAC = 62,      /* HCLK_MASK1[30] */
	CLOCK_IOBUS_RESERVED = 63,  /* HCLK_MASK1[31] */
	CLOCK_IOBUS_WDT = 64,       /* HCLK_MASK2[0] */
	CLOCK_IOBUS_GPIO = 65,      /* HCLK_MASK2[1] */
	CLOCK_IOBUS_CMU = 66,       /* HCLK_MASK2[2] */
	CLOCK_IOBUS_SYSSM = 67,     /* HCLK_MASK2[3] */
	CLOCK_IOBUS_MAX = 68
} CLOCKIobus_t;

/* Peripheral Clocks */
typedef enum CLOCK_PERI { /* Peri. Name */
	// MICOM Peri
	CLOCK_PERI_SFMC = 0,
	CLOCK_PERI_CAN0 = 1,
	CLOCK_PERI_CAN1 = 2,
	CLOCK_PERI_CAN2 = 3,
	CLOCK_PERI_GPSB0 = 4,
	CLOCK_PERI_GPSB1 = 5,
	CLOCK_PERI_GPSB2 = 6,
	CLOCK_PERI_GPSB3 = 7,
	CLOCK_PERI_GPSB4 = 8,
	CLOCK_PERI_UART0 = 9,
	CLOCK_PERI_UART1 = 10,
	CLOCK_PERI_UART2 = 11,
	CLOCK_PERI_UART3 = 12,
	CLOCK_PERI_UART4 = 13,
	CLOCK_PERI_UART5 = 14,
	CLOCK_PERI_I2C0 = 15,
	CLOCK_PERI_I2C1 = 16,
	CLOCK_PERI_I2C2 = 17,
	CLOCK_PERI_I2C3 = 18,
	CLOCK_PERI_I2C4 = 19,
	CLOCK_PERI_I2C5 = 20,
	CLOCK_PERI_PWM0 = 21,
	CLOCK_PERI_PWM1 = 22,
	CLOCK_PERI_PWM2 = 23,
	CLOCK_PERI_ICTC0 = 24,
	CLOCK_PERI_ICTC1 = 25,
	CLOCK_PERI_ICTC2 = 26,
	CLOCK_PERI_ICTC3 = 27,
	CLOCK_PERI_ICTC4 = 28,
	CLOCK_PERI_ICTC5 = 29,
	CLOCK_PERI_ADC0 = 30,
	CLOCK_PERI_ADC1 = 31,
	CLOCK_PERI_TIMER0 = 32,
	CLOCK_PERI_TIMER1 = 33,
	CLOCK_PERI_TIMER2 = 34,
	CLOCK_PERI_TIMER3 = 35,
	CLOCK_PERI_TIMER4 = 36,
	CLOCK_PERI_TIMER5 = 37,
	CLOCK_PERI_TIMER6 = 38,
	CLOCK_PERI_TIMER7 = 39,
	CLOCK_PERI_TIMER8 = 40,
	CLOCK_PERI_TIMER9 = 41,
	CLOCK_PERI_I2S0 = 42,
	CLOCK_PERI_I2S1 = 43,
	CLOCK_PERI_GMAC0 = 44,
	CLOCK_PERI_GMAC1 = 45,
	CLOCK_PERI_MAX = 46
} CLOCKPeri_t;

// MICOM Register Offsets
#define CLOCK_MCKC_PLL0PMS  (0x000)
#define CLOCK_MCKC_PLL1PMS  (0x00C)
#define CLOCK_MCKC_CLKDIVC  (0x018)
#define CLOCK_MCKC_CLKCTRL  (0x020) //   need to modify VCP code //(0x01C)
#define CLOCK_MCKC_PCLKCTRL (0x028) //(0x024)

// MICOM Subsystem Register Offsets
#define CLOCK_MCKC_HCLK0    (0x000)
#define CLOCK_MCKC_HCLK1    (0x004)
#define CLOCK_MCKC_HCLK2    (0x008)
#define CLOCK_MCKC_HCLKSWR0 (0x00C)
#define CLOCK_MCKC_HCLKSWR1 (0x010)
#define CLOCK_MCKC_HCLKSWR2 (0x014)

/*
 * MICOM CLKCTRL Configuration Register
 */
#define CLOCK_MCLKCTRL_SEL_SHIFT (4)
#define CLOCK_MCLKCTRL_SEL_MASK  (0x7)

#define CLOCK_MCLKCTRL_CONFIG_MIN   (1)
#define CLOCK_MCLKCTRL_CONFIG_MAX   (15)
#define CLOCK_MCLKCTRL_CONFIG_SHIFT (5)
#define CLOCK_MCLKCTRL_CONFIG_MASK  (0xF)
#define CLOCK_MCLKCTRL_EN_SHIFT     (22) // Caution : Do NOT change this value to LOW.
#define CLOCK_MCLKCTRL_DIVSTS_SHIFT (29)
#define CLOCK_MCLKCTRL_CLKCHG_SHIFT (31)

/*
 * PLL Configuration Register
 */
#define CLOCK_PLL_MAX_RATE      (3200000000) // Max. 3200MHz
#define CLOCK_PLL_MIN_RATE      (25000000)   // Min.   25MHz
#define CLOCK_PLL_VCO_MAX       (3200000000) // Max. 3200MHz
#define CLOCK_PLL_VCO_MIN       (1600000000) // Min. 1600MHz
#define CLOCK_PLL_P_MAX         (6)          // 63   FREF = FIN/p  (4MHz ~ 12MHz)
#define CLOCK_PLL_P_MIN         (2)          // 1    FREF = FIN/p  (4MHz ~ 12MHz)
#define CLOCK_PLL_P_SHIFT       (0)
#define CLOCK_PLL_P_MASK        (0x3F)
#define CLOCK_PLL_M_MAX         (1023)
#define CLOCK_PLL_M_MIN         (64)
#define CLOCK_PLL_M_SHIFT       (6)
#define CLOCK_PLL_M_MASK        (0x1FF)
#define CLOCK_PLL_S_MAX         (6)
#define CLOCK_PLL_S_MIN         (0)
#define CLOCK_PLL_S_SHIFT       (15)
#define CLOCK_PLL_S_MASK        (0x7)
#define CLOCK_PLL_SRC_SHIFT     (18)
#define CLOCK_PLL_SRC_MASK      (0x3)
#define CLOCK_PLL_BYPASS_SHIFT  (20)
#define CLOCK_PLL_LOCKST_SHIFT  (21)
#define CLOCK_PLL_CHGPUMP_SHIFT (22)
#define CLOCK_PLL_CHGPUMP_MASK  (0x3)
#define CLOCK_PLL_LOCKEN_SHIFT  (24)
#define CLOCK_PLL_RSEL_SHIFT    (25)
#define CLOCK_PLL_RSEL_MASK     (0xF)
#define CLOCK_PLL_EN_SHIFT      (31)

/*
 * PCLKCTRL Configuration Register
 */
#define CLOCK_PCLKCTRL_MAX_FCKS     (1600000000)
#define CLOCK_PCLKCTRL_DIV_MIN      (0)
#define CLOCK_PCLKCTRL_DIV_SHIFT    (0)
#define CLOCK_PCLKCTRL_DIV_XXX_MAX  (0xFFF)
#define CLOCK_PCLKCTRL_DIV_XXX_MASK (CLOCK_PCLKCTRL_DIV_XXX_MAX)
#define CLOCK_PCLKCTRL_DIV_YYY_MAX  (0xFFFFFF)
#define CLOCK_PCLKCTRL_DIV_YYY_MASK (CLOCK_PCLKCTRL_DIV_YYY_MAX)
#define CLOCK_PCLKCTRL_SEL_MIN      (0)
#define CLOCK_PCLKCTRL_SEL_MAX      (28)
#define CLOCK_PCLKCTRL_SEL_SHIFT    (24)
#define CLOCK_PCLKCTRL_SEL_MASK     (0x1F)
#define CLOCK_PCLKCTRL_EN_SHIFT     (29)
#define CLOCK_PCLKCTRL_OUTEN_SHIFT  (30)
#define CLOCK_PCLKCTRL_MD_SHIFT     (31)

/* CLKCTRL SEL */
typedef enum CLOCK_M_CLK_CTRL_SEL {
	CLOCK_MCLKCTRL_SEL_XIN = 0,
	CLOCK_MCLKCTRL_SEL_PLL0 = 1,
	CLOCK_MCLKCTRL_SEL_PLL1 = 2,
	CLOCK_MCLKCTRL_SEL_XINDIV = 3,
	CLOCK_MCLKCTRL_SEL_PLL0DIV = 4,
	CLOCK_MCLKCTRL_SEL_PLL1DIV = 5,
	CLOCK_MCLKCTRL_SEL_EXTIN0 = 6,
	CLOCK_MCLKCTRL_SEL_EXTIN1 = 7
} CLOCKMclkCtrlSel_t;

/* PCLK Mode Selection */
typedef enum CLOCK_PCLK_CTRL_MODE {
	CLOCK_PCLKCTRL_MODE_DCO = 0,
	CLOCK_PCLKCTRL_MODE_DIVIDER = 1,
	CLOCK_PCLKCTRL_MODE_MAX = 2
} CLOCKPclkCtrlMode_t;

typedef enum CLOCK_M_PCLK_CTRL_SEL {
	CLOCK_MPCLKCTRL_SEL_PLL0 = 0,
	CLOCK_MPCLKCTRL_SEL_PLL1 = 1,
	CLOCK_MPCLKCTRL_SEL_XIN = 5,
	CLOCK_MPCLKCTRL_SEL_PLL0DIV = 10,
	CLOCK_MPCLKCTRL_SEL_PLL1DIV = 11,
	CLOCK_MPCLKCTRL_SEL_XINDIV = 23,
	CLOCK_MPCLKCTRL_SEL_EXTIN0 = 25,
	CLOCK_MPCLKCTRL_SEL_EXTIN1 = 26,
	CLOCK_MPCLKCTRL_SEL_EXTIN2 = 27,
	CLOCK_MPCLKCTRL_SEL_EXTIN3 = 28
} CLOCKMpclkCtrlSel_t;

/* PCLK Type */
typedef enum CLOCK_PCLK_CTRL_TYPE {
	CLOCK_PCLKCTRL_TYPE_XXX = 0,
	CLOCK_PCLKCTRL_TYPE_YYY = 1,
	CLOCK_PCLKCTRL_TYPE_MAX = 2
} CLOCKPclkCtrlType_t;

typedef struct CLOCK_CLK_CTRL {
	uint32_t uiFreq;
	uint32_t uiEn;
	uint32_t uiConf;
	uint32_t uiSel;
} CLOCKClkCtrl_t;

typedef struct CLOCK_PCLK_CTRL {
	uint32_t uiPeriName;
	uint32_t uiFreq;
	uint32_t uiMd;
	uint32_t uiEn;
	uint32_t uiSel;
	uint32_t uiDivVal;
} CLOCKPclkCtrl_t;

void CLOCK_Init(void);

signed long CLOCK_SetPllRate(signed long iId, uint32_t uiRate);

uint32_t CLOCK_GetPllRate(signed long iId);

signed long CLOCK_SetPllDiv(signed long iId, uint32_t uiPllDiv);

signed long CLOCK_SetClkCtrlRate(signed long iId, uint32_t uiRate);

uint32_t CLOCK_GetClkCtrlRate(signed long iId);

signed long CLOCK_IsPeriEnabled(signed long iId);

signed long CLOCK_EnablePeri(signed long iId);

signed long CLOCK_DisablePeri(signed long iId);

uint32_t CLOCK_GetPeriRate(signed long iId);

signed long CLOCK_SetPeriRate(signed long iId, uint32_t uiRate);

signed long CLOCK_IsIobusPwdn(signed long iId);

signed long CLOCK_EnableIobus(signed long iId, boolean bEn);

signed long CLOCK_SetIobusPwdn(signed long iId, boolean bEn);

signed long CLOCK_SetSwReset(signed long iId, boolean bReset);

static uint32_t stMicomClockSource[CLOCK_SRC_MAX_NUM];

static void CLOCK_DevWritePll(uint32_t uiReg, uint32_t uiEn, uint32_t uiP, uint32_t uiM,
			      uint32_t uiS);

static void CLOCK_DevWritePclkCtrl(uint32_t uiReg, uint32_t uiMd, uint32_t uiEn, uint32_t uiSel,
				   uint32_t uiDiv, uint32_t uiType);

static void CLOCK_DevWriteClkCtrl(uint32_t uiReg, uint32_t uiEn, uint32_t uiConf, uint32_t uiSel);

static signed long CLOCK_DevFindPms(CLOCKPms_t *psPll, uint32_t uiSrcFreq);

static signed long CLOCK_DevSetPllRate(uint32_t uiReg, uint32_t uiRate);

static uint32_t CLOCK_DevGetPllRate(uint32_t uiReg);

static uint32_t CLOCK_DevGetPllDiv(signed long iId);

static signed long CLOCK_DevFindClkCtrl(CLOCKClkCtrl_t *CLKCTRL);

static uint32_t CLOCK_DevCalPclkDiv(const CLOCKPclkCtrl_t *psPclkCtrl, uint32_t *puiClkDiv,
				    const uint32_t uiSrcClk,
				    /*uint32_t                              uiDivMin,*/
				    uint32_t uiDivMax);

static signed long CLOCK_DevFindPclk(CLOCKPclkCtrl_t *psPclkCtrl, CLOCKPclkCtrlType_t eType);

#define TIMER_INDEX CONFIG_TCC_VCPTTC_TIMER_INDEX

#define TIMER_IRQ            DT_INST_IRQN(0)
#define TIMER_BASE_ADDR      DT_INST_REG_ADDR(0)
#define TIMER_CLOCK_FREQUECY DT_INST_PROP(0, clock_frequency)

#define TICKS_PER_SEC   CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CYCLES_PER_SEC  TIMER_CLOCK_FREQUECY
#define CYCLES_PER_TICK (CYCLES_PER_SEC / TICKS_PER_SEC)

#define CYCLES_NEXT_MIN (10000)
#define CYCLES_NEXT_MAX (XTTC_MAX_INTERVAL_COUNT)

BUILD_ASSERT(TIMER_CLOCK_FREQUECY == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	     "Configured system timer frequency does not match the TTC "
	     "clock frequency in the device tree");

BUILD_ASSERT(CYCLES_PER_SEC >= TICKS_PER_SEC,
	     "Timer clock frequency must be greater than the system tick "
	     "frequency");

BUILD_ASSERT((CYCLES_PER_SEC % TICKS_PER_SEC) == 0,
	     "Timer clock frequency is not divisible by the system tick "
	     "frequency");

#define MCU_BSP_TIMER_BASE (0xA0F2A000UL)

typedef enum TIMERStartMode {
	TIMER_START_MAINCNT = 0x0UL,
	TIMER_START_ZERO = 0x1UL,

} TIMERStartMode_t;

typedef enum TIMERChannel {
	TIMER_CH_0 = 0UL,
	TIMER_CH_1 = 1UL,
	TIMER_CH_2 = 2UL,
	TIMER_CH_3 = 3UL,
	TIMER_CH_4 = 4UL,
	TIMER_CH_5 = 5UL,
	TIMER_CH_6 = 6UL,
	TIMER_CH_7 = 7UL,
	TIMER_CH_8 = 8UL,
	TIMER_CH_9 = 9UL,
	TIMER_CH_MAX = 10UL,

} TIMERChannel_t;

typedef enum TIMERCounterMode {
	TIMER_COUNTER_MAIN = 0UL,
	TIMER_COUNTER_COMP0 = 1UL,
	TIMER_COUNTER_COMP1 = 2UL,
	TIMER_COUNTER_SMALL_COMP = 3UL,

} TIMERCounterMode_t;

typedef enum TIMEROperationMode {
	TIMER_OP_FREERUN = 0UL,
	TIMER_OP_ONESHOT

} TIMEROpMode_t;

typedef void (*TIMERHandler)(TIMERChannel_t uiChannel, const void *pArgs);

typedef struct TIMERCfgTable {
	TIMERChannel_t ctChannel;
	TIMERStartMode_t ctStartMode;
	TIMEROpMode_t ctOpMode;
	TIMERCounterMode_t ctCounterMode;
	uint32_t ctMainValueUsec;
	uint32_t ctCmp0ValueUsec;
	uint32_t ctCmp1ValueUsec;
	TIMERHandler fnHandler;
	void *pArgs;

} TIMERConfig_t;

#define SAL_DRVID_TMR   (1900)
#define SAL_MAX_INT_VAL (4294967295UL)

enum {
	TIMER_ERR_ALREADY_DONE = (uint32_t)SAL_DRVID_TMR,
	TIMER_ERR_SET_CLOCK = (uint32_t)SAL_DRVID_TMR + 1UL,
	TIMER_ERR_NOT_INITIALIZED = (uint32_t)SAL_DRVID_TMR + 2UL,
	TIMER_ERR_INVALID_PARAM = (uint32_t)SAL_DRVID_TMR + 3UL,
	TIMER_ERR_OUTOF_RANGE_CHANNEL = (uint32_t)SAL_DRVID_TMR + 4UL,
	TIMER_ERR_OUTOF_COMPARATOR_0 = (uint32_t)SAL_DRVID_TMR + 5UL,
	TIMER_ERR_OUTOF_COMPARATOR_1 = (uint32_t)SAL_DRVID_TMR + 6UL,
	TIMER_ERR_NOT_SUPPORT_MODE = (uint32_t)SAL_DRVID_TMR + 7UL,
	TIMER_ERR_INVALID_TIME = (uint32_t)SAL_DRVID_TMR + 8UL,
	TIMER_ERR_OUTOF_MAINCNT = (uint32_t)SAL_DRVID_TMR + 9UL

}; // TIMERErrorCode_t

typedef struct TIMERResourceTable {
	TIMERChannel_t rtChannel;
	boolean rtUsed;
	TIMERHandler rtHandler;
	void *rtArgs;

} TIMERResource_t;

/* Register Map */
#define TMR_OP_EN_CFG    (0x000UL)
#define TMR_MAIN_CNT_LVD (0x004UL)
#define TMR_CMP_VALUE0   (0x008UL)
#define TMR_CMP_VALUE1   (0x00CUL)
#define TMR_PSCL_CNT     (0x010UL)
#define TMR_MAIN_CNT     (0x014UL)
#define TMR_IRQ_CTRL     (0x018UL)

/* Configuration Value */
#define TMR_OP_EN_CFG_LDM0_ON         (1UL << 28UL)
#define TMR_OP_EN_CFG_LDM1_ON         (1UL << 29UL)
#define TMR_OP_EN_CFG_OPMODE_FREE_RUN (0UL << 26UL)
#define TMR_OP_EN_CFG_OPMODE_ONE_SHOT (1UL << 26UL)
#define TMR_OP_EN_CFG_LDZERO_OFFSET   (25UL)
#define TMR_OP_EN_CFG_CNT_EN          (1UL << 24UL)

/* 0: Reading this register to be cleared, 1: Writing a non-zero value to MASKED_IRQ_STATUS to be
 * cleared */
#define TMR_IRQ_CLR_CTRL_WRITE (1UL << 31UL)
#define TMR_IRQ_CLR_CTRL_READ  (0UL << 31UL)
#define TMR_IRQ_MASK_ALL       (0x1FUL)
#define TMR_IRQ_CTRL_IRQ_EN0   (1UL << 16UL)
#define TMR_IRQ_CTRL_IRQ_EN1   (2UL << 16UL)
#define TMR_IRQ_CTRL_IRQ_EN2   (4UL << 16UL)
#define TMR_IRQ_CTRL_IRQ_EN3   (8UL << 16UL)
#define TMR_IRQ_CTRL_IRQ_EN4   (16UL << 16UL)
#define TMR_IRQ_CTRL_IRQ_ALLEN                                                                     \
	((TMR_IRQ_CTRL_IRQ_EN0) | (TMR_IRQ_CTRL_IRQ_EN1) | (TMR_IRQ_CTRL_IRQ_EN2) |                \
	 (TMR_IRQ_CTRL_IRQ_EN3) | (TMR_IRQ_CTRL_IRQ_EN4))

#define TMR_PRESCALE (11UL)
#define TMR_CLK_RATE ((12UL) * (1000UL) * (1000UL))

#endif
