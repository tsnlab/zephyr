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

void BSP_PreInit(void)
{
	CLOCK_Init();
	GIC_Init();

	//	PMU_Init();

	//	BSP_EnableSYSPower();
}
