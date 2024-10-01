/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <cmsis_core.h>
#include "soc.h"

void z_arm_platform_init(void)
{
	/*
	 * Use normal exception vectors address range (0x0-0x1C).
	 */
#if 0
	unsigned int sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_V_Msk;
	__set_SCTLR(sctlr);
#endif
}

static uint32_t stMicomClockSource[CLOCK_SRC_MAX_NUM];

/*
 * sal_internal
 */

static SALRetCode_t FR_CoreMB(void)
{
	__asm__("    DSB");
	__asm__("    ISB");

	return SAL_RET_SUCCESS;
}

static SALRetCode_t FR_CoreDiv64To32(unsigned long long *pullDividend, uint32_t uiDivisor,
				     uint32_t *puiRem)
{
	SALRetCode_t retVal = SAL_RET_SUCCESS;
	unsigned long long rem = 0;
	unsigned long long b = uiDivisor;
	unsigned long long d = 1;
	unsigned long long res = 0;
	uint32_t high = 0;

	if (pullDividend != NULL_PTR) {
		rem = *pullDividend;
		high = (uint32_t)(rem >> 32ULL);

		/* Reduce the thing a bit first */
		if (high >= uiDivisor) {
			high /= uiDivisor;
			res = ((unsigned long long)high) << 32ULL;

			// CERT-C Integers (CERT INT30-C) : Ensure that unsigned integer operations
			// do not wrap
			if ((uiDivisor > 0UL) &&
			    ((rem / (unsigned long long)uiDivisor) >= (unsigned long long)high)) {
				retVal = SAL_RET_FAILED;
			} else {
				rem -= (((unsigned long long)high * (unsigned long long)uiDivisor)
					<< 32ULL);
			}
		}

		if (retVal == SAL_RET_SUCCESS) {
			while (((b > 0ULL) && (b < rem))) {
				b = b + b;
				d = d + d;
			}

			do {
				if (rem >= b) {
					rem -= b;

					if ((0xFFFFFFFFFFFFFFFFULL - d) < res) {
						retVal = SAL_RET_FAILED;
						break;
					} else {
						res += d;
					}
				}

				b >>= 1UL;
				d >>= 1UL;
			} while (d != 0ULL);

			if (retVal == SAL_RET_SUCCESS) {
				*pullDividend = res;
			}
		}
	} else {
		retVal = SAL_RET_FAILED;
	}

	if (puiRem != NULL_PTR) {
		*puiRem = (uint32_t)rem;
	}

	return retVal;
}

/*
 * clock_dev start
 */

static uint32_t CLOCK_DevGetPllRate(uint32_t uiReg)
{
	uint32_t uiRegVa;
	CLOCKPms_t sPllCfg;
	unsigned long long ullTemp;

	if (uiReg == 0UL) {
		return 0;
	} else {
		uiRegVa = sys_read32(uiReg);
	}

	sPllCfg.uiP = (uiRegVa >> (uint32_t)CLOCK_PLL_P_SHIFT) & ((uint32_t)CLOCK_PLL_P_MASK);
	sPllCfg.uiM = (uiRegVa >> (uint32_t)CLOCK_PLL_M_SHIFT) & ((uint32_t)CLOCK_PLL_M_MASK);
	sPllCfg.uiS = (uiRegVa >> (uint32_t)CLOCK_PLL_S_SHIFT) & ((uint32_t)CLOCK_PLL_S_MASK);
	sPllCfg.uiEn = (uiRegVa >> (uint32_t)CLOCK_PLL_EN_SHIFT) & (1UL);
	sPllCfg.uiSrc = (uiRegVa >> (uint32_t)CLOCK_PLL_SRC_SHIFT) & ((uint32_t)CLOCK_PLL_SRC_MASK);
	ullTemp = (unsigned long long)CLOCK_XIN_CLK_RATE * (unsigned long long)sPllCfg.uiM;
	(void)FR_CoreDiv64To32(&ullTemp, (uint32_t)(sPllCfg.uiP), NULL);

	return (uint32_t)((ullTemp) >> sPllCfg.uiS);
}

uint32_t CLOCK_GetPllRate(signed long iId)
{
	uint32_t uiReg;
	CLOCKPll_t uiPllId;

	uiReg = (uint32_t)0;
	uiPllId = (CLOCKPll_t)iId;

	if (uiPllId == CLOCK_PLL_MICOM_0) {
		uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PLL0PMS;
	} else if (uiPllId == CLOCK_PLL_MICOM_1) {
		uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PLL1PMS;
	} else {
		return 0;
	}

	return CLOCK_DevGetPllRate(uiReg);
}

static uint32_t CLOCK_DevGetPllDiv(signed long iId)
{
	uint32_t uiRet;
	CLOCKMpll_t uiMpllId;
	uint32_t uiReg;
	uint32_t uiOffSet;
	uint32_t uiRegVa;

	uiRet = 0;
	uiMpllId = (CLOCKMpll_t)iId;
	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_CLKDIVC;
	uiRegVa = 0;

	switch (uiMpllId) {
	case CLOCK_MPLL_0:
	case CLOCK_MPLL_1: {
		uiOffSet = (3UL - (uint32_t)iId) * 8UL;
		break;
	}

	case CLOCK_MPLL_XIN: {
		uiOffSet = 8UL;
		break;
	}

	default: {
		uiOffSet = 0UL;
		break;
	}
	}

	uiRegVa = sys_read32(uiReg);

	if (uiOffSet != 0UL) {
		uiRet = ((uiRegVa >> uiOffSet) & 0x3FUL);
	}

	return uiRet;
}

static void CLOCK_DevResetClkSrc(signed long iId)
{
	if (iId >= (signed long)CLOCK_SRC_MAX_NUM) {
		return;
	}

	if (iId < (signed long)CLOCK_PLL_MAX_NUM) {
		stMicomClockSource[iId] = CLOCK_GetPllRate(iId);
		stMicomClockSource[CLOCK_PLL_MAX_NUM + iId] =
			stMicomClockSource[iId] / (CLOCK_DevGetPllDiv(iId) + 1UL);
	}
}

void CLOCK_Init(void)
{
	static signed long iInitialized = 0;
	signed long iIdx;

	if (iInitialized == 1L) {
		return;
	}

	iInitialized = 1;

	for (iIdx = 0L; iIdx < ((signed long)CLOCK_PLL_MAX_NUM * 2L); iIdx++) {
		stMicomClockSource[iIdx] = 0;
	}

	stMicomClockSource[CLOCK_PLL_MAX_NUM * 2] = (uint32_t)CLOCK_XIN_CLK_RATE;
	stMicomClockSource[(CLOCK_PLL_MAX_NUM * 2) + 1] = 0UL;

	for (iIdx = 0L; iIdx < (signed long)CLOCK_PLL_MAX_NUM; iIdx++) {
		CLOCK_DevResetClkSrc(iIdx);
	}
}

/*
 * gic start
 */

void GIC_Init(void)
{
	unsigned long uiRegOffset;

	uiRegOffset = 0;

	GIC_DIST->dCTRL &= (unsigned long)(~ARM_BIT_GIC_DIST_ICDDCR_EN);
	GIC_DIST->dCTRL |= (unsigned long)ARM_BIT_GIC_DIST_ICDDCR_EN;

	for (; uiRegOffset <= ((unsigned long)(GIC_INT_SRC_CNT - 1UL) / 4UL); uiRegOffset++) {
		GIC_DIST->dIPRIORITYRn[uiRegOffset] = 0xA0A0A0A0UL;
	}

	GIC_CPU->cPMR = 0xFFUL;
	GIC_CPU->cCTLR |= GIC_CPUIF_CTRL_ENABLEGRP0;
	(void)FR_CoreMB(); // Origin code 20240930 POOKY (void)SAL_CoreMB();

	return;
}

static boolean gIsPOR = (FALSE);

static void PMU_FmuCtrl(PMUFmu_t tFmu)
{
	unsigned long uiFieldOffset;
	unsigned long uiEn;

	uiFieldOffset = 0UL;
	uiEn = 0UL;

	if ((tFmu > PMU_FMU_NOTHING) && (tFmu < PMU_FMU_MAX)) {
		switch (tFmu) {
		case PMU_FMU_XIN_PVT_DISABLE: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN;
			uiEn = 0UL;
			break;
		}
		case PMU_FMU_XIN_PVT_EDGE: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN;
			uiEn = 2UL;
			break;
		}
		case PMU_FMU_XIN_PVT_LEVEL: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_XIN_PERIODIC_EN;
			uiEn = 3UL;
			break;
		}
		case PMU_FMU_PWR_PVT_DISABLE: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN;
			uiEn = 0UL;
			break;
		}
		case PMU_FMU_PWR_PVT_EDGE: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN;
			uiEn = 2UL;
			break;
		}
		case PMU_FMU_PWR_PVT_LEVEL: {
			uiFieldOffset = PMU_ADDR_PVT_SM_CFG_FIELD_POWER_PERIODIC_EN;
			uiEn = 3UL;
			break;
		}
		default: {
			break;
		}
		}

		PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
			    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

		PMU_REG_APPEND(uiEn, uiFieldOffset, PMU_VA_MASK_2 << uiFieldOffset,
			       PMU_ADDR_PVT_SM_CFG);
	} else {
		// PMU_E("Unknown reset type %d\n", (unsigned long)tFmu);
	}
}

void PMU_Init(void)
{
	if (PMU_REG_GET(0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_RST_STATUS0) == 0UL) {
		gIsPOR = TRUE;
	}

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_SET(0xFFFFFFFF, 0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_RST_STATUS0);

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_SET(0xFFFFFFFF, 0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_RST_STATUS1);

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_SET(0x0, 0x0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_HSM_RSTN_MSK);

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_APPEND(0x0, PMU_ADDR_PVT4_CFG_FIELD_XIN_MON_EN,
		       PMU_ADDR_PVT4_CFG_FIELD_XIN_MON_EN_MASK, PMU_ADDR_PVT4_CFG);

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_SET(0x0, 0x0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_PVT0_CFG);

	PMU_REG_SET(PMU_VA_WR_PASS, PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW,
		    PMU_ADDR_PMU_WR_PW_FIELD_PMU_WR_PW_MASK, PMU_ADDR_PMU_WR_PW);

	PMU_REG_SET(0x0, 0x0, PMU_ADDR_COMMON_FIELD_FULL_MASK, PMU_ADDR_RST_ENABLE);

	PMU_FmuCtrl(PMU_FMU_XIN_PVT_LEVEL);
	PMU_FmuCtrl(PMU_FMU_PWR_PVT_LEVEL);
}

SALRetCode_t GPIO_Set(unsigned long uiPort, unsigned long uiData)
{
	unsigned long bit;
	unsigned long data_or;
	unsigned long data_bic;
	SALRetCode_t ret;

	ret = SAL_RET_SUCCESS;

	bit = (unsigned long)1 << (uiPort & GPIO_PIN_MASK);

	if (uiData > 1UL) {
		ret = SAL_RET_FAILED;
	} else {
		/* set data */
		if (uiData != 0UL) {
			data_or = GPIO_REG_DATA_OR(uiPort);
			sys_write32(bit, data_or);
		} else {
			data_bic = GPIO_REG_DATA_BIC(uiPort);
			sys_write32(bit, data_bic);
		}
	}

	return ret;
}

static void GPIO_SetRegister(unsigned long addr, unsigned long bit, unsigned long enable)
{
	unsigned long base_val;
	unsigned long set_val;

	base_val = sys_read32(addr);
	set_val = 0UL;

	if (enable == 1UL) {
		set_val = (base_val | bit);
	} else if (enable == 0UL) {
		set_val = (base_val & ~bit);
	} else {
		// Do nothing.
	}

	sys_write32(set_val, addr);
}

SALRetCode_t GPIO_Config(unsigned long uiPort, unsigned long uiConfig)
{
	unsigned long pin;
	unsigned long bit;
	unsigned long func;
	unsigned long pull;
	unsigned long ds;
	unsigned long ien;
	unsigned long base_val;
	unsigned long comp_val;
	unsigned long set_val;
	unsigned long reg_fn;
	unsigned long pullen_addr;
	unsigned long pullsel_addr;
	unsigned long cd_addr;
	unsigned long outen_addr;
	unsigned long ien_addr;
	SALRetCode_t ret;

	ret = SAL_RET_SUCCESS;
	pin = uiPort & (unsigned long)GPIO_PIN_MASK;
	bit = (unsigned long)1 << pin;
	func = uiConfig & (unsigned long)GPIO_FUNC_MASK;
	pull = uiConfig & ((unsigned long)GPIO_PULL_MASK << (unsigned long)GPIO_PULL_SHIFT);
	ds = uiConfig & ((unsigned long)GPIO_DS_MASK << (unsigned long)GPIO_DS_SHIFT);
	ien = uiConfig & ((unsigned long)GPIO_INPUTBUF_MASK << (unsigned long)GPIO_INPUTBUF_SHIFT);

	/* function */
	reg_fn = GPIO_REG_FN(uiPort, pin);
	base_val = sys_read32(reg_fn) &
		   (~((unsigned long)0xF << ((pin % (unsigned long)8) * (unsigned long)4)));
	set_val = base_val | (func << ((pin % (unsigned long)8) * (unsigned long)4));
	sys_write32(set_val, reg_fn);
	/* configuration check */
	comp_val = sys_read32(reg_fn);

	if (comp_val != set_val) {
		ret = SAL_RET_FAILED;
	} else {
		/* pull-up/down */
		if (pull == GPIO_PULLUP) {
			if (GPIO_IS_GPIOK(uiPort)) {
				pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
			} else {
				pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
			}

			GPIO_SetRegister(pullen_addr, bit, (unsigned long)TRUE);

			if (GPIO_IS_GPIOK(uiPort)) {
				pullsel_addr = (GPIO_PMGPIO_BASE + 0x14UL);
			} else {
				pullsel_addr = (GPIO_REG_BASE(uiPort) + 0x20UL);
			}

			GPIO_SetRegister(pullsel_addr, bit, (unsigned long)TRUE);
		} else if (pull == GPIO_PULLDN) {
			if (GPIO_IS_GPIOK(uiPort)) {
				pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
			} else {
				pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
			}

			GPIO_SetRegister(pullen_addr, bit, (unsigned long)TRUE);

			if (GPIO_IS_GPIOK(uiPort)) {
				pullsel_addr = (GPIO_PMGPIO_BASE + 0x14UL);
			} else {
				pullsel_addr = (GPIO_REG_BASE(uiPort) + 0x20UL);
			}

			GPIO_SetRegister(pullsel_addr, bit, (unsigned long)FALSE);
		} else {
			if (GPIO_IS_GPIOK(uiPort)) {
				pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
			} else {
				pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
			}

			GPIO_SetRegister(pullen_addr, bit, (unsigned long)FALSE);
		}

		/* drive strength */
		if (ds != 0UL) {
			if (GPIO_IS_GPIOK(uiPort)) {
				cd_addr = (GPIO_PMGPIO_BASE + 0x18UL) +
					  (0x4UL * ((pin) / (unsigned long)16));
			} else {
				cd_addr = (GPIO_REG_BASE(uiPort) + 0x14UL) +
					  (0x4UL * ((pin) / (unsigned long)16));
			}

			ds = ds >> (unsigned long)GPIO_DS_SHIFT;
			base_val = sys_read32(cd_addr) &
				   ~((unsigned long)3
				     << ((pin % (unsigned long)16) * (unsigned long)2));
			set_val = base_val | ((ds & (unsigned long)0x3)
					      << ((pin % (unsigned long)16) * (unsigned long)2));
			sys_write32(set_val, cd_addr);
		}

		/* direction */
		if ((uiConfig & VCP_GPIO_OUTPUT) != 0UL) {
			outen_addr = GPIO_REG_OUTEN(uiPort);

			GPIO_SetRegister(outen_addr, bit, (unsigned long)TRUE);
		} else {
			outen_addr = GPIO_REG_OUTEN(uiPort);

			GPIO_SetRegister(outen_addr, bit, (unsigned long)FALSE);
		}

		/* input buffer enable */
		if (ien == GPIO_INPUTBUF_EN) {
			if (GPIO_IS_GPIOK(uiPort)) {
				ien_addr = (GPIO_PMGPIO_BASE + 0x0CUL);
			} else {
				ien_addr = (GPIO_REG_BASE(uiPort) + 0x24UL);
			}

			GPIO_SetRegister(ien_addr, bit, (unsigned long)TRUE);
		} else if (ien == GPIO_INPUTBUF_DIS) {
			if (GPIO_IS_GPIOK(uiPort)) {
				ien_addr = (GPIO_PMGPIO_BASE + 0x0CUL);
			} else {
				ien_addr = (GPIO_REG_BASE(uiPort) + 0x24UL);
			}

			GPIO_SetRegister(ien_addr, bit, (unsigned long)FALSE);
		} else // QAC
		{
			; // no statement
		}
	}

	return ret;
}

void BSP_EnableSYSPower(void)
{
	/* Enable SYS_3P3 */
	(void)GPIO_Config(SYS_PWR_EN, (unsigned long)(GPIO_FUNC(0U) | VCP_GPIO_OUTPUT));
	(void)GPIO_Set(SYS_PWR_EN, 1UL);
}

void BSP_PreInit(void)
{
	CLOCK_Init();
	GIC_Init();

	PMU_Init();

	BSP_EnableSYSPower();
}
