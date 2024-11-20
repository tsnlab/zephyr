/*
***************************************************************************************************
*
*   FileName : clock_dev.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*
*   TCC Version 1.0
*
*   This source code contains confidential information of Telechips.
*
*   Any unauthorized use without a written permission of Telechips including not limited to
*   re-distribution in source or binary form is strictly prohibited.
*
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty of
*   merchantability, fitness for a particular purpose or non-infringement of any patent, copyright
*   or other third party intellectual property right. No warranty is made, express or implied,
*   regarding the information's accuracy,completeness, or performance.
*
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between
*   Telechips and Company.
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty
*   (of merchantability, fitness for a particular purpose or non-infringement of any patent,
*   copyright or other third party intellectual property right. No warranty is made, express or
*   implied, regarding the information's accuracy, completeness, or performance.
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement
*   between Telechips and Company.
*
***************************************************************************************************
*/

#ifndef TCC_SOC_CLOCK_DEV_HEADER
#define TCC_SOC_CLOCK_DEV_HEADER

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

#define CLOCK_XIN_CLK_RATE (12000000UL) // 12MHz

/* PLL channel index */
typedef enum CLOCK_PLL {
	CLOCK_PLL_MICOM_0 = 0,
	CLOCK_PLL_MICOM_1 = 1
} CLOCKPll_t;

/* MICOM pll source channel index */
typedef enum CLOCK_M_PLL {
	CLOCK_MPLL_0 = 0,
	CLOCK_MPLL_1 = 1,
	CLOCK_MPLL_DIV_0 = 2,
	CLOCK_MPLL_DIV_1 = 3,
	CLOCK_MPLL_XIN = 4
} CLOCKMpll_t;

typedef enum CLOCK_PCLK_CTRL_SEL {
	CLOCK_PCLKCTRL_SEL_PLL0 = 0,
	CLOCK_PCLKCTRL_SEL_PLL1 = 1,
	CLOCK_PCLKCTRL_SEL_XIN = 5,
	CLOCK_PCLKCTRL_SEL_PLL0DIV = 10,
	CLOCK_PCLKCTRL_SEL_PLL1DIV = 11,
	CLOCK_PCLKCTRL_SEL_XINDIV = 23
} CLOCKPclkctrlSel_t;

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

typedef struct CLOCK_PMS {
	uint32_t uiFpll;
	uint32_t uiEn;
	uint32_t uiP;
	uint32_t uiM;
	uint32_t uiS;
	uint32_t uiSrc;
} CLOCKPms_t;

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

#endif // TCC_SOC_CLOCK_DEV_HEADER
