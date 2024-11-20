/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _BOARD_SOC__H_
#define _BOARD_SOC__H_

#include <zephyr/drivers/interrupt_controller/intc_tic.h>

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
#define SALDisabled (FALSE)
#define SALEnabled  (TRUE)

/*
 * Return Codes
 */
typedef enum SALRetCode {
	SAL_RET_SUCCESS = 0,
	SAL_RET_FAILED = 1

} SALRetCode_t;

typedef SALRetCode_t (*SALCoreMB)(void);

SALRetCode_t SAL_CoreMB (void);
SALRetCode_t SAL_MemSet(void *pMem, unsigned char ucValue, uint32_t uiSize);
SALRetCode_t SAL_CoreDiv64To32(unsigned long long *pullDividend, uint32_t uiDivisor,
                   uint32_t *puiRem);
/* pmu start */

#if 0
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
#endif

typedef unsigned char boolean; /* for use with TRUE/FALSE        */

#define MCU_BSP_PMIO_BASE (0xA0F28800UL)
#define MCU_BSP_GPIO_BASE (0xA0F22000UL)

#define GPIO_PMGPIO_BASE (MCU_BSP_PMIO_BASE)

#define GPIO_REG_BASE(x)                                                                           \
	(MCU_BSP_GPIO_BASE + ((((x)&GPIO_PORT_MASK) >> (unsigned long)GPIO_PORT_SHIFT) * 0x40UL))

#define GPIO_IS_GPIOK(x) (boolean)((((x)&GPIO_PORT_MASK) == GPIO_PORT_K) ? 1 : 0)

#define GPIO_REG_DATA(x)     (GPIO_REG_BASE(x) + 0x00UL)
#define GPIO_REG_OUTEN(x)    (GPIO_REG_BASE(x) + 0x04UL)
#define GPIO_REG_DATA_OR(x)  (GPIO_REG_BASE(x) + 0x08UL)
#define GPIO_REG_DATA_BIC(x) (GPIO_REG_BASE(x) + 0x0CUL)
#define GPIO_REG_PULLEN(x)                                                                         \
	(GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x10UL) : (GPIO_REG_BASE(x) + 0x1CUL))
#define GPIO_REG_PULLSEL(x)                                                                        \
	(GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x14UL) : (GPIO_REG_BASE(x) + 0x20UL))
#define GPIO_REG_CD(x, pin)                                                                        \
	((GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x18UL) : (GPIO_REG_BASE(x) + 0x14UL)) +          \
	 (0x4UL * ((pin) / (unsigned long)16)))
#define GPIO_REG_IEN(x)                                                                            \
	(GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x0CUL) : (GPIO_REG_BASE(x) + 0x24UL))
//#define GPIO_REG_IS(x)                  (GPIO_REG_BASE(x) + 0x28UL)
//#define GPIO_REG_SR(x)                  (GPIO_REG_BASE(x) + 0x2CUL)
#define GPIO_REG_FN(x, pin) ((GPIO_REG_BASE(x) + 0x30UL) + (0x4UL * ((pin) / (unsigned long)8)))
#define GPIO_MFIO_CFG       (MCU_BSP_GPIO_BASE + (0x2B4UL))
#define GPIO_PERICH_SEL     (MCU_BSP_GPIO_BASE + (0x2B8UL))

#define GPIO_PMGPIO_SEL (GPIO_PMGPIO_BASE + 0x8UL)

#define GPIO_LIST_NUM (6)

#define GPIO_INPUTBUF_SHIFT (10)
#define GPIO_INPUTBUF_MASK  (0x3UL)
#define GPIO_INPUTBUF_EN    ((2UL | 1UL) << (unsigned long)GPIO_INPUTBUF_SHIFT)
#define GPIO_INPUTBUF_DIS   ((2UL | 0UL) << (unsigned long)GPIO_INPUTBUF_SHIFT)

#define GPIO_OUTPUT_SHIFT (9)
#define VCP_GPIO_OUTPUT   (1UL << (unsigned long)GPIO_OUTPUT_SHIFT)
//#define GPIO_INPUT        (0UL << (unsigned long)GPIO_OUTPUT_SHIFT)

#define GPIO_DS_SHIFT (6)
#define GPIO_DS_MASK  (0x7UL)
#define GPIO_DS(x)    ((((x) & (unsigned long)GPIO_DS_MASK) | 0x4UL) << (unsigned long)GPIO_DS_SHIFT)

#define GPIO_PULL_SHIFT (4)
#define GPIO_PULL_MASK  (0x3UL)
#define GPIO_NOPULL     (0UL << (unsigned long)GPIO_PULL_SHIFT)
#define GPIO_PULLUP     (1UL << (unsigned long)GPIO_PULL_SHIFT)
#define GPIO_PULLDN     (2UL << (unsigned long)GPIO_PULL_SHIFT)

#define GPIO_FUNC_MASK (0xFUL)
#define GPIO_FUNC(x)   ((x) & (unsigned long)GPIO_FUNC_MASK)

#define GPIO_PIN_MASK     (0x1FUL)
#define GPIO_PIN_NUM_MASK (0x3FUL) // original 1FUL , avoid code sonar warning

#define GPIO_PORT_SHIFT (5)
#define GPIO_PORT_MASK  ((unsigned long)0x1F << (unsigned long)GPIO_PORT_SHIFT)

//(n<<GPIO_PORT_SHIFT)                  n = ofset/0x40
#define GPIO_PORT_A ((unsigned long)0 << (unsigned long)GPIO_PORT_SHIFT) // offset: 0x000
#define GPIO_PORT_B ((unsigned long)1 << (unsigned long)GPIO_PORT_SHIFT) // offset: 0x040
#define GPIO_PORT_C ((unsigned long)2 << (unsigned long)GPIO_PORT_SHIFT) // offset: 0x080
#define GPIO_PORT_K ((unsigned long)3 << (unsigned long)GPIO_PORT_SHIFT) // offset: 0x0c0

#define GPIO_GPA(x) (GPIO_PORT_A | ((x) & (unsigned long)0x1F))
#define GPIO_GPB(x) (GPIO_PORT_B | ((x) & (unsigned long)0x1F))
#define GPIO_GPC(x) (GPIO_PORT_C | ((x) & (unsigned long)0x1F))
#define GPIO_GPK(x) (GPIO_PORT_K | ((x) & (unsigned long)0x1F))

#define GPIO_GP_MAX GPIO_GPK((unsigned long)0x1f)

#define SYS_PWR_EN (GPIO_GPC(2UL))

//#define TEST_LED_BLINK (GPIO_GPK(16UL))
#define TEST_LED_BLINK (GPIO_GPB(4UL))

#endif /* _BOARD_SOC__H_ */
