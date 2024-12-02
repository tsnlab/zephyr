/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef CLOCK_CONTROL_TCCVCP_HEADER
#define CLOCK_CONTROL_TCCVCP_HEADER

#define CLOCK_PLL_MAX_NUM (2)
#define CLOCK_SRC_MAX_NUM (6) /*  ((CLOCK_PLL_MAX_NUM * 2 ) + 2) */

#define CLOCK_XIN_CLK_RATE (12000000UL) /* 12MHz */

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
	/* MICOM Peri */
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

/*
 * CKC Register Offsets
 */

/* MICOM Register Offsets */
#define CLOCK_MCKC_PLL0PMS  (0x000)
#define CLOCK_MCKC_PLL1PMS  (0x00C)
#define CLOCK_MCKC_CLKDIVC  (0x018)
#define CLOCK_MCKC_CLKCTRL  (0x020)
#define CLOCK_MCKC_PCLKCTRL (0x028)

/* MICOM Subsystem Register Offsets */
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
#define CLOCK_MCLKCTRL_EN_SHIFT     (22)
#define CLOCK_MCLKCTRL_DIVSTS_SHIFT (29)
#define CLOCK_MCLKCTRL_CLKCHG_SHIFT (31)

/*
 * PLL Configuration Register
 */
#define CLOCK_PLL_MAX_RATE      (3200000000) /* Max. 3200MHz */
#define CLOCK_PLL_MIN_RATE      (25000000)   /* Min.   25MHz */
#define CLOCK_PLL_VCO_MAX       (3200000000) /* Max. 3200MHz */
#define CLOCK_PLL_VCO_MIN       (1600000000) /* Min. 1600MHz */
#define CLOCK_PLL_P_MAX         (6)          /* 63   FREF = FIN/p  (4MHz ~ 12MHz) */
#define CLOCK_PLL_P_MIN         (2)          /* 1    FREF = FIN/p  (4MHz ~ 12MHz) */
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
enum clock_mclk_ctrl_sel {
	CLOCK_MCLKCTRL_SEL_XIN = 0,
	CLOCK_MCLKCTRL_SEL_PLL0 = 1,
	CLOCK_MCLKCTRL_SEL_PLL1 = 2,
	CLOCK_MCLKCTRL_SEL_XINDIV = 3,
	CLOCK_MCLKCTRL_SEL_PLL0DIV = 4,
	CLOCK_MCLKCTRL_SEL_PLL1DIV = 5,
	CLOCK_MCLKCTRL_SEL_EXTIN0 = 6,
	CLOCK_MCLKCTRL_SEL_EXTIN1 = 7
};

/* PCLK Type */
enum clock_pclk_ctrl_type {
	CLOCK_PCLKCTRL_TYPE_XXX = 0,
	CLOCK_PCLKCTRL_TYPE_YYY = 1,
	CLOCK_PCLKCTRL_TYPE_MAX = 2
};

/* PCLK Mode Selection */
enum clock_pclk_ctrl_mode {
	CLOCK_PCLKCTRL_MODE_DCO = 0,
	CLOCK_PCLKCTRL_MODE_DIVIDER = 1,
	CLOCK_PCLKCTRL_MODE_MAX = 2
};

enum clock_mpclk_ctrl_sel {
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
};

struct clock_pms {
	uint32_t fpll;
	uint32_t en;
	uint32_t p;
	uint32_t m;
	uint32_t s;
	uint32_t src;
};

struct clock_clk_ctrl {
	uint32_t freq;
	uint32_t en;
	uint32_t conf;
	uint32_t sel;
};

struct clock_pclk_ctrl {
	uint32_t peri_name;
	uint32_t freq;
	uint32_t md;
	uint32_t en;
	uint32_t sel;
	uint32_t div_val;
};

#endif /* CLOCK_CONTROL_TCCVCP_HEADER */
