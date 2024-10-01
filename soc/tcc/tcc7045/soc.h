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

typedef SALRetCode_t (*SALCoreMB)(void);

#if 0
/* ARM Core */
typedef struct SALCoreTable {
	SALCoreMB fnCoreMB;
	SALCoreRMB fnCoreRMB;
	SALCoreWMB fnCoreWMB;
	SALCoreCriticalEnter fnCoreCriticalEnter;
	SALCoreCriticalExit fnCoreCriticalExit;
	SALCoreWaitForEvent fnCoreWaitForEvent;
	SALCoreDiv64To32 fnCoreDiv64To32;

} SALCoreFunc_t;
#endif

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

/* gic start */
typedef struct GICDistributor {
	uint32_t dCTRL;  /* GICD_CTLR        Distributor Control Register.                        */
	uint32_t dTYPER; /* GICD_TYPER       Interrupt Controller Type Register.                  */
	uint32_t dIIDR;  /* GICD_IIDR        Distributor Implementer Identification Register.     */
	uint32_t dRSVD1[29];        /* Reserved.	*/
	uint32_t dIGROUPRn[32];     /* GICD_IGROUPRn    Interrupt Security Registers.     */
	uint32_t dISENABLERn[32];   /* GICD_ISENABLERn  Interrupt Set-Enable Registers.   */
	uint32_t dICENABLERn[32];   /* GICD_ICENABLERn  Interrupt Clear-Enable Registers.   */
	uint32_t dISPENDRn[32];     /* GICD_ISPENDRn    Interrupt Set-Pending Registers.     */
	uint32_t dICPENDRn[32];     /* GICD_ICPENDRn    Interrupt Clear-Pending Registers.     */
	uint32_t dISACTIVERn[32];   /* GICD_ISACTIVERn  Interrupt Set-Active Registers.   */
	uint32_t dICACTIVERn[32];   /* GICD_ICACTIVERn  Interrupt Clear-Active Registers.   */
	uint32_t dIPRIORITYRn[255]; /* GICD_IPRIORITYRn Interrupt Priority Registers. */
	uint32_t dRSVD3[1];         /* Reserved.	 */
	uint32_t dITARGETSRn[255];  /* GICD_ITARGETSRn  Interrupt Processor Target Registers.  */
	uint32_t dRSVD4[1];         /* Reserved.	 */
	uint32_t dICFGRn[64];       /* GICD_ICFGRn      Interrupt Configuration Registers.       */
	uint32_t dPPISR[1];         /* GICD_PPISR.	 */
	uint32_t dSPISRn[15];       /* GICD_SPISRn       */
	uint32_t dRSVD5[112];       /* Reserved.       */
	uint32_t dSGIR; /* GICD_SGIR        Software Generate Interrupt Register.                */
	uint32_t dRSVD6[3];      /* Reserved.      */
	uint32_t dCPENDSGIRn[4]; /* GICD_CPENDSGIRn  SGInterrupt Clear-Active Registers. */
	uint32_t dSPENDSGIRn[4]; /* GICD_SPENDSGIRn  SGInterrupt Set-Active Registers. */
} GICDistributor_t;

typedef struct GICCpuInterface {
	uint32_t cCTLR;  /* GICC_CTLR        CPU Interface Control Register.                      */
	uint32_t cPMR;   /* GICC_PMR         Interrupt Priority Mask Register.                    */
	uint32_t cBPR;   /* GICC_BPR         Binary Point Register.                               */
	uint32_t cIAR;   /* GICC_IAR         Interrupt Acknowledge Register.                      */
	uint32_t cEOIR;  /* GICC_EOIR        End Interrupt Register.                              */
	uint32_t cRPR;   /* GICC_RPR         Running Priority Register.                           */
	uint32_t cHPPIR; /* GICC_HPPIR       Highest Pending Interrupt Register.                  */
	uint32_t cABPR;  /* GICC_ABPR        Aliased Binary Point Register.                       */
	uint32_t cAIAR;  /* GICC_AIAR        Aliased Interrupt Acknowledge Register               */
	uint32_t cAEOIR; /* GICC_AEOIR       Aliased End Interrupt Register.                      */
	uint32_t cAHPPIR;   /* GICC_AHPPIR      Aliased Highest Pending Interrupt Register.   */
	uint32_t cRSVD[52]; /* Reserved. */
	uint32_t cIIDR; /* GICC_IIDR        CPU Interface Identification Register.               */
} GICCpuInterface_t;

#define MCU_BSP_GIC_BASE (0xA0F10000UL)

/* 0x1b903000u GICC_DIR  */
#define VCP_GIC_DIST_BASE (MCU_BSP_GIC_BASE + 0x1000UL)
#define VCP_GIC_CPU_BASE  (MCU_BSP_GIC_BASE + 0x2000UL)

#define GIC_DIST ((volatile GICDistributor_t *)(VCP_GIC_DIST_BASE))
#define GIC_CPU  ((volatile GICCpuInterface_t *)(VCP_GIC_CPU_BASE))

/* ----------- DISTRIBUTOR CONTROL REGISTER -----------                 */
#define ARM_BIT_GIC_DIST_ICDDCR_EN (0x00000001UL) /* Global GIC enable. */

#define GIC_CPUIF_CTRL_ENABLEGRP0 (0x00000001UL) /* Enable secure interrupts.      */
#define GIC_CPUIF_CTRL_ENABLEGRP1 (0x00000002UL) /* Enable non-secure interrupts.  */
#define GIC_CPUIF_CTRL_ACKCTL     (0x00000004UL) /* Secure ack of NS interrupts.   */

#define GIC_SPI_START (32UL)

enum {
	GIC_CAN0_0 = (GIC_SPI_START + 0UL),
	GIC_CAN0_1 = (GIC_SPI_START + 1UL),
	GIC_CAN1_0 = (GIC_SPI_START + 2UL),
	GIC_CAN1_1 = (GIC_SPI_START + 3UL),
	GIC_CAN2_0 = (GIC_SPI_START + 4UL),
	GIC_CAN2_1 = (GIC_SPI_START + 5UL),
	GIC_GPSB = (GIC_SPI_START + 6UL),
	GIC_GPSB1 = (GIC_SPI_START + 7UL),
	GIC_GPSB2 = (GIC_SPI_START + 8UL),
	GIC_GPSB3 = (GIC_SPI_START + 9UL),
	GIC_GPSB4 = (GIC_SPI_START + 10UL),
	GIC_GPSB0_DMA = (GIC_SPI_START + 11UL),
	GIC_GPSB1_DMA = (GIC_SPI_START + 12UL),
	GIC_GPSB2_DMA = (GIC_SPI_START + 13UL),
	GIC_GPSB3_DMA = (GIC_SPI_START + 14UL),
	GIC_GPSB4_DMA = (GIC_SPI_START + 15UL),
	GIC_UART0 = (GIC_SPI_START + 16UL),
	GIC_UART1 = (GIC_SPI_START + 17UL),
	GIC_UART2 = (GIC_SPI_START + 18UL),
	GIC_UART3 = (GIC_SPI_START + 19UL),
	GIC_UART4 = (GIC_SPI_START + 20UL),
	GIC_UART5 = (GIC_SPI_START + 21UL),
	GIC_DMA0 = (GIC_SPI_START + 22UL),
	GIC_DMA1 = (GIC_SPI_START + 23UL),
	GIC_DMA2 = (GIC_SPI_START + 24UL),
	GIC_DMA3 = (GIC_SPI_START + 25UL),
	GIC_DMA4 = (GIC_SPI_START + 26UL),
	GIC_DMA5 = (GIC_SPI_START + 27UL),
	GIC_DMA6 = (GIC_SPI_START + 28UL),
	GIC_DMA7 = (GIC_SPI_START + 29UL),
	GIC_I2C = (GIC_SPI_START + 30UL),
	GIC_I2C1 = (GIC_SPI_START + 31UL),
	GIC_I2C2 = (GIC_SPI_START + 32UL),
	GIC_I2C3 = (GIC_SPI_START + 33UL),
	GIC_I2C4 = (GIC_SPI_START + 34UL),
	GIC_I2C5 = (GIC_SPI_START + 35UL),
	GIC_VS_I2C0 = (GIC_SPI_START + 36UL),
	GIC_VS_I2C1 = (GIC_SPI_START + 37UL),
	GIC_VS_I2C2 = (GIC_SPI_START + 38UL),
	GIC_VS_I2C3 = (GIC_SPI_START + 39UL),
	GIC_VS_I2C4 = (GIC_SPI_START + 40UL),
	GIC_VS_I2C5 = (GIC_SPI_START + 41UL),
	GIC_ICTC0 = (GIC_SPI_START + 42UL),
	GIC_ICTC1 = (GIC_SPI_START + 43UL),
	GIC_ICTC2 = (GIC_SPI_START + 44UL),
	GIC_ICTC3 = (GIC_SPI_START + 45UL),
	GIC_ICTC4 = (GIC_SPI_START + 46UL),
	GIC_ICTC5 = (GIC_SPI_START + 47UL),
	GIC_RED_ICTC0 = (GIC_SPI_START + 48UL),
	GIC_RED_ICTC1 = (GIC_SPI_START + 49UL),
	GIC_RED_ICTC2 = (GIC_SPI_START + 50UL),
	GIC_RED_ICTC3 = (GIC_SPI_START + 51UL),
	GIC_RED_ICTC4 = (GIC_SPI_START + 52UL),
	GIC_RED_ICTC5 = (GIC_SPI_START + 53UL),
	GIC_ADC0 = (GIC_SPI_START + 54UL),
	GIC_ADC1 = (GIC_SPI_START + 55UL),
	GIC_TIMER_0 = (GIC_SPI_START + 56UL),
	GIC_TIMER_1 = (GIC_SPI_START + 57UL),
	GIC_TIMER_2 = (GIC_SPI_START + 58UL),
	GIC_TIMER_3 = (GIC_SPI_START + 59UL),
	GIC_TIMER_4 = (GIC_SPI_START + 60UL),
	GIC_TIMER_5 = (GIC_SPI_START + 61UL),
	GIC_TIMER_6 = (GIC_SPI_START + 62UL),
	GIC_TIMER_7 = (GIC_SPI_START + 63UL),
	GIC_TIMER_8 = (GIC_SPI_START + 64UL),
	GIC_TIMER_9 = (GIC_SPI_START + 65UL),
	GIC_WATCHDOG = (GIC_SPI_START + 66UL),
	GIC_PMU_WATCHDOG = (GIC_SPI_START + 67UL),
	GIC_DEFAULT_SLV_ERR = (GIC_SPI_START + 68UL),
	GIC_SFMC = (GIC_SPI_START + 69UL),
	GIC_CR5_PMU = (GIC_SPI_START + 70UL),
	GIC_EXT0 = (GIC_SPI_START + 71UL),
	GIC_EXT1 = (GIC_SPI_START + 72UL),
	GIC_EXT2 = (GIC_SPI_START + 73UL),
	GIC_EXT3 = (GIC_SPI_START + 74UL),
	GIC_EXT4 = (GIC_SPI_START + 75UL),
	GIC_EXT5 = (GIC_SPI_START + 76UL),
	GIC_EXT6 = (GIC_SPI_START + 77UL),
	GIC_EXT7 = (GIC_SPI_START + 78UL),
	GIC_EXT8 = (GIC_SPI_START + 79UL),
	GIC_EXT9 = (GIC_SPI_START + 80UL),
	GIC_EXTn0 = (GIC_SPI_START + 81UL),
	GIC_EXTn1 = (GIC_SPI_START + 82UL),
	GIC_EXTn2 = (GIC_SPI_START + 83UL),
	GIC_EXTn3 = (GIC_SPI_START + 84UL),
	GIC_EXTn4 = (GIC_SPI_START + 85UL),
	GIC_EXTn5 = (GIC_SPI_START + 86UL),
	GIC_EXTn6 = (GIC_SPI_START + 87UL),
	GIC_EXTn7 = (GIC_SPI_START + 88UL),
	GIC_EXTn8 = (GIC_SPI_START + 89UL),
	GIC_EXTn9 = (GIC_SPI_START + 90UL),
	GIC_FMU_IRQ = (GIC_SPI_START + 91UL),
	GIC_FMU_FIQ = (GIC_SPI_START + 92UL),
	GIC_I2S_DMA = (GIC_SPI_START + 93UL),
	GIC_GMAC = (GIC_SPI_START + 94UL),
	GIC_GMAC_TX0 = (GIC_SPI_START + 95UL),
	GIC_GMAC_TX1 = (GIC_SPI_START + 96UL),
	GIC_GMAC_RX0 = (GIC_SPI_START + 97UL),
	GIC_GMAC_RX1 = (GIC_SPI_START + 98UL),
	GIC_SRAM_ECC = (GIC_SPI_START + 99UL),
	GIC_DMAC_ = (GIC_SPI_START + 100UL),
	GIC_DMAC_INT_CLEAR = (GIC_SPI_START + 101UL),
	GIC_DMAC_INTERR = (GIC_SPI_START + 102UL),
	GIC_HSM_WDT_OUTPUT = (GIC_SPI_START + 103UL),
	GIC_MBOX_SLV_TX = (GIC_SPI_START + 104UL),
	GIC_MBOX_MST_TX = (GIC_SPI_START + 105UL),
	GIC_AES = (GIC_SPI_START + 106UL),
	GIC_TRNG = (GIC_SPI_START + 107UL),
	GIC_TRNG_FAD = (GIC_SPI_START + 108UL),
	GIC_PKE = (GIC_SPI_START + 109UL),
	GIC_PKE_ECC_S = (GIC_SPI_START + 110UL),
	GIC_PKE_ECC_D = (GIC_SPI_START + 111UL),
	GIC_HSM_DEFAULT_SLV_ERR = (GIC_SPI_START + 112UL),
	GIC_HSM_CPU_SYS_RESET_REQ = (GIC_SPI_START + 113UL),
	GIC_PFLASH = (GIC_SPI_START + 114UL),
	GIC_DFLASH = (GIC_SPI_START + 115UL),
	GIC_RTC_ALARM = (GIC_SPI_START + 116UL),
	GIC_RTC_WAKEUP = (GIC_SPI_START + 117UL),
	GIC_GMAC_TX2 = (GIC_SPI_START + 118UL),
	GIC_GMAC_RX2 = (GIC_SPI_START + 119UL),
	GIC_SFMC1 = (GIC_SPI_START + 120UL),
	GIC_SPU = (GIC_SPI_START + 121UL),
};

#define GIC_INT_SRC_CNT (GIC_SPU + 1UL)

/* pmu start */

#define PMU_VA_WR_PASS (0x5AFEACE5UL)

typedef enum {
	PMU_FMU_NOTHING = 0,

	PMU_FMU_XIN_PVT_DISABLE = 1,
	PMU_FMU_XIN_PVT_EDGE = 2,
	PMU_FMU_XIN_PVT_LEVEL = 3,

	PMU_FMU_PWR_PVT_DISABLE = 4,
	PMU_FMU_PWR_PVT_EDGE = 5,
	PMU_FMU_PWR_PVT_LEVEL = 6,

	PMU_FMU_MAX = 7
} PMUFmu_t;

#define PMU_VA_MASK_1  (0x1UL)
#define PMU_VA_MASK_2  (0x3UL)
#define PMU_VA_MASK_3  (0x7UL)
#define PMU_VA_MASK_4  (0xFUL)
#define PMU_VA_MASK_5  (0x1FUL)
#define PMU_VA_MASK_6  (0x3FUL)
#define PMU_VA_MASK_7  (0x7FUL)
#define PMU_VA_MASK_8  (0xFFUL)
#define PMU_VA_MASK_9  (0x1FFUL)
#define PMU_VA_MASK_10 (0x3FFUL)
#define PMU_VA_MASK_11 (0x7FFUL)
#define PMU_VA_MASK_12 (0xFFFUL)
#define PMU_VA_MASK_13 (0x1FFFUL)
#define PMU_VA_MASK_14 (0x3FFFUL)
#define PMU_VA_MASK_15 (0x7FFFUL)
#define PMU_VA_MASK_16 (0xFFFFUL)
#define PMU_VA_MASK_17 (0x1FFFFUL)
#define PMU_VA_MASK_18 (0x3FFFFUL)
#define PMU_VA_MASK_19 (0x7FFFFUL)
#define PMU_VA_MASK_20 (0xFFFFFUL)
#define PMU_VA_MASK_21 (0x1FFFFFUL)
#define PMU_VA_MASK_22 (0x3FFFFFUL)
#define PMU_VA_MASK_23 (0x7FFFFFUL)
#define PMU_VA_MASK_24 (0xFFFFFFUL)
#define PMU_VA_MASK_25 (0x1FFFFFFUL)
#define PMU_VA_MASK_26 (0x3FFFFFFUL)
#define PMU_VA_MASK_27 (0x7FFFFFFUL)
#define PMU_VA_MASK_28 (0xFFFFFFFUL)
#define PMU_VA_MASK_29 (0x1FFFFFFFUL)
#define PMU_VA_MASK_30 (0x3FFFFFFFUL)
#define PMU_VA_MASK_31 (0x7FFFFFFFUL)
#define PMU_VA_MASK_32 (0xFFFFFFFFUL)

#define PMU_ADDR_BASE              (0xA0F28000)
#define PMU_ADDR_GLB_CONFIG        (PMU_ADDR_BASE + 0x00UL)
#define PMU_ADDR_RST_STATUS0       (PMU_ADDR_BASE + 0x04UL)
#define PMU_ADDR_RST_STATUS1       (PMU_ADDR_BASE + 0x08UL)
#define PMU_ADDR_RST_ENABLE        (PMU_ADDR_BASE + 0x0CUL)
#define PMU_ADDR_COLD_RST_REQ      (PMU_ADDR_BASE + 0x10UL)
#define PMU_ADDR_WARM_RST_REQ      (PMU_ADDR_BASE + 0x14UL)
#define PMU_ADDR_HSM_RSTN_MSK      (PMU_ADDR_BASE + 0x18UL)
#define PMU_ADDR_HSM_SWRSTN        (PMU_ADDR_BASE + 0x1CUL)
#define PMU_ADDR_HSM_STATUS        (PMU_ADDR_BASE + 0x20UL)
#define PMU_ADDR_SYSRST_CTRL       (PMU_ADDR_BASE + 0x24UL)
#define PMU_ADDR_MEM_ECC_CFG       (PMU_ADDR_BASE + 0x28UL)
#define PMU_ADDR_PVT0_CFG          (PMU_ADDR_BASE + 0x2CUL)
#define PMU_ADDR_PVT1_CFG          (PMU_ADDR_BASE + 0x30UL)
#define PMU_ADDR_PVT2_CFG          (PMU_ADDR_BASE + 0x34UL)
#define PMU_ADDR_PVT3_CFG          (PMU_ADDR_BASE + 0x38UL)
#define PMU_ADDR_PVT4_CFG          (PMU_ADDR_BASE + 0x3CUL)
#define PMU_ADDR_PVT_SM_CFG        (PMU_ADDR_BASE + 0x40UL)
#define PMU_ADDR_XIN_SELECT_FREQ   (PMU_ADDR_BASE + 0x44UL)
#define PMU_ADDR_OSC_CFG           (PMU_ADDR_BASE + 0x48UL)
#define PMU_ADDR_ECID_USER0_FBOUT0 (PMU_ADDR_BASE + 0x4CUL)
#define PMU_ADDR_ECID_USER0_FBOUT1 (PMU_ADDR_BASE + 0x50UL)
#define PMU_ADDR_ECID_USER0_FBOUT2 (PMU_ADDR_BASE + 0x54UL)
#define PMU_ADDR_ECID_USER1_FBOUT0 (PMU_ADDR_BASE + 0x58UL)
#define PMU_ADDR_ECID_USER1_FBOUT1 (PMU_ADDR_BASE + 0x5CUL)
#define PMU_ADDR_ECID_USER1_FBOUT2 (PMU_ADDR_BASE + 0x60UL)
#define PMU_ADDR_MEM_CFG           (PMU_ADDR_BASE + 0x64UL)
#define PMU_ADDR_BACKUP_REG        (PMU_ADDR_BASE + 0x68UL)
#define PMU_ADDR_PMU_WDT_EN        (PMU_ADDR_BASE + 0x6CUL)
#define PMU_ADDR_PMU_WDT_CLR       (PMU_ADDR_BASE + 0x70UL)
#define PMU_ADDR_PMU_WDT_IRQ_CNT   (PMU_ADDR_BASE + 0x74UL)
#define PMU_ADDR_PMU_WDT_RST_CNT   (PMU_ADDR_BASE + 0x78UL)
#define PMU_ADDR_PMU_WDT_SM_MODE   (PMU_ADDR_BASE + 0x7CUL)
#define PMU_ADDR_PMU_WDT_LOCK      (PMU_ADDR_BASE + 0x80UL)
#define PMU_ADDR_PMU_WR_PW         (PMU_ADDR_BASE + 0x3FCUL)

#define PMU_ADDR_COMMON_FIELD_ZERO       (0UL)
#define PMU_ADDR_COMMON_FIELD_FULL_MASK  (PMU_VA_MASK_31)
#define PMU_ADDR_COMMON_FIELD_CLEAR_MASK (0UL)

/* FIELD INFO (Register: PVT_SM_CFG) ===========================================*/
#define PMU_ADDR_PVT_SM_CFG_FIELD_POWER_SM_FMU_EN (3UL)
#define PMU_ADDR_PVT_SM_CFG_FIELD_POWER_SM_FMU_EN_MASK                                             \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT_SM_CFG_FIELD_POWER_SM_FMU_EN)
#define PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN (2UL)
#define PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN_MASK                                           \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN)
#define PMU_ADDR_PVT_SM_CFG_FIELD_XIN_SM_FMU_EN (1UL)
#define PMU_ADDR_PVT_SM_CFG_FIELD_XIN_SM_FMU_EN_MASK                                               \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT_SM_CFG_FIELD_XIN_SM_FMU_EN)
#define PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN (0UL)
#define PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN_MASK                                             \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN)

/* FIELD INFO (Register: PVT4_CFG) ===========================================*/
#define PMU_ADDR_PVT4_CFG_FIELD_XIN_MON_EN (16UL)
#define PMU_ADDR_PVT4_CFG_FIELD_XIN_MON_EN_MASK                                                    \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT4_CFG_FIELD_XIN_MON_EN)
#define PMU_ADDR_PVT4_CFG_FIELD_CFG_XIN_MON_SETTLE (13UL)
#define PMU_ADDR_PVT4_CFG_FIELD_CFG_XIN_MON_SETTLE_MASK                                            \
	(PMU_VA_MASK_3 << PMU_ADDR_PVT4_CFG_FIELD_CFG_XIN_MON_SETTLE)
#define PMU_ADDR_PVT4_CFG_FIELD_XIN_OURANGE (12UL)
#define PMU_ADDR_PVT4_CFG_FIELD_XIN_OURANGE_MASK                                                   \
	(PMU_VA_MASK_1 << PMU_ADDR_PVT4_CFG_FIELD_XIN_OURANGE)
#define PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_CAPTURE (8UL)
#define PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_CAPTURE_MASK                                               \
	(PMU_VA_MASK_4 << PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_CAPTURE)
#define PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_RANGE (0UL)
#define PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_RANGE_MASK                                                 \
	(PMU_VA_MASK_8 << PMU_ADDR_PVT4_CFG_FIELD_OSC_XIN_RANGE)

/* FIELD INFO (Register: BACKUP_REG) ===========================================*/
#define PMU_ADDR_BACKUP_REG_FIELD_BACKUP_REG (0UL)
#define PMU_ADDR_BACKUP_REG_FIELD_BACKUP_REG_MASK                                                  \
	(PMU_VA_MASK_32 << PMU_ADDR_BACKUP_REG_FIELD_BACKUP_REG)

/* FIELD INFO (Register: PMU_WDT_EN) ===========================================*/
#define PMU_ADDR_PMU_WDT_EN_FIELD_WDT_EN      (0UL)
#define PMU_ADDR_PMU_WDT_EN_FIELD_WDT_EN_MASK (PMU_VA_MASK_1 << PMU_ADDR_PMU_WDT_EN_FIELD_WDT_EN)

/* FIELD INFO (Register: PMU_WDT_CLR) ===========================================*/
#define PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_RST_CLR (1UL)
#define PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_RST_CLR_MASK                                                \
	(PMU_VA_MASK_1 << PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_RST_CLR)
#define PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_CLR (0UL)
#define PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_CLR_MASK                                                    \
	(PMU_VA_MASK_1 << PMU_ADDR_PMU_WDT_CLR_FIELD_WDT_CLR)

/* FIELD INFO (Register: PMU_WDT_IRQ_CNT) ===========================================*/
#define PMU_ADDR_PMU_WDT_IRQ_CNT_FIELD_WDT_IRQ_VALUE (0UL)
#define PMU_ADDR_PMU_WDT_IRQ_CNT_FIELD_WDT_IRQ_VALUE_MASK                                          \
	(PMU_VA_MASK_32 << PMU_ADDR_PMU_WDT_IRQ_CNT_FIELD_WDT_IRQ_VALUE)

/* FIELD INFO (Register: PMU_WDT_RST_CNT) ===========================================*/
#define PMU_ADDR_PMU_WDT_RST_CNT_FIELD_WDT_RST_CNT (0UL)
#define PMU_ADDR_PMU_WDT_RST_CNT_FIELD_WDT_RST_CNT_MASK                                            \
	(PMU_VA_MASK_32 << PMU_ADDR_PMU_WDT_RST_CNT_FIELD_WDT_RST_CNT)

/* FIELD INFO (Register: PMU_WDT_SM_MODE) ===========================================*/
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_CONT (3UL)
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_CONT_MASK                                                   \
	(PMU_VA_MASK_1 << PMU_ADDR_PMU_WDT_SM_MODE_FIELD_CONT)
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_RST_SEL (2UL)
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_RST_SEL_MASK                                                \
	(PMU_VA_MASK_1 << PMU_ADDR_PMU_WDT_SM_MODE_FIELD_RST_SEL)
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_2oo3 (0UL)
#define PMU_ADDR_PMU_WDT_SM_MODE_FIELD_2oo3_MASK                                                   \
	(PMU_VA_MASK_2 << PMU_ADDR_PMU_WDT_SM_MODE_FIELD_2oo3)

/* FIELD INFO (Register: PMU_WDT_LOCK) ===========================================*/
#define PMU_ADDR_PMU_WDT_LOCK_FIELD_PMU_WDT_LOCK (0UL)
#define PMU_ADDR_PMU_WDT_LOCK_FIELD_PMU_WDT_LOCK_MASK                                              \
	(PMU_VA_MASK_32 << PMU_ADDR_PMU_WDT_LOCK_FIELD_PMU_WDT_LOCK)

/* FIELD INFO (Register: PMU_WR_PW) ===========================================*/
#define PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW (0UL)
#define PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK                                                    \
	(PMU_VA_MASK_32 << PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW)

#define PMU_REG_GET(o, m, a)                                                                       \
	(((*(volatile unsigned long *)((unsigned long)a)) & (unsigned long)m) >> (unsigned long)o)
#define PMU_REG_SET(x, o, m, a)                                                                    \
	(*(volatile unsigned long *)((unsigned long)a)) =                                          \
		(((unsigned long)x << (unsigned long)o) & (unsigned long)m)
#define PMU_REG_APPEND(x, o, m, a)                                                                 \
	((*(volatile unsigned long *)((unsigned long)a)) =                                         \
		 (((*(volatile unsigned long *)((unsigned long)a)) & (~((unsigned long)m))) |      \
		  ((unsigned long)x << o)))

#endif /* _BOARD__H_ */
