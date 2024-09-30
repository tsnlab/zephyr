/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _BOARD__H_
#define _BOARD__H_

/* Define CMSIS configurations */
#define __CR_REV 1U

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

/* sal_com start */
/*
 * Return Codes
 */
typedef enum SALRetCode {
	SAL_RET_SUCCESS = 0,
	SAL_RET_FAILED = 1

} SALRetCode_t;

/* clock_reg start */

#define MCU_BSP_CKC_BASE    (0xA0F24000UL)
#define MCU_BSP_SUBSYS_BASE (0xA0F20000UL)

#define CLOCK_BASE_ADDR (MCU_BSP_CKC_BASE)

#define CLOCK_XIN_CLK_RATE (12000000UL) // 12MHz

#define CLOCK_PLL_MAX_NUM (2)
#define CLOCK_SRC_MAX_NUM (6) //  ((CLOCK_PLL_MAX_NUM * 2 ) + 2)

// MICOM Register Offsets
#define CLOCK_MCKC_PLL0PMS  (0x000)
#define CLOCK_MCKC_PLL1PMS  (0x00C)
#define CLOCK_MCKC_CLKDIVC  (0x018)
#define CLOCK_MCKC_CLKCTRL  (0x020) //   need to modify VCP code //(0x01C)
#define CLOCK_MCKC_PCLKCTRL (0x028) //(0x024)

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

typedef struct CLOCK_PMS {
	uint32_t uiFpll;
	uint32_t uiEn;
	uint32_t uiP;
	uint32_t uiM;
	uint32_t uiS;
	uint32_t uiSrc;
} CLOCKPms_t;

/* clock_reg end */

#endif /* _BOARD__H_ */
