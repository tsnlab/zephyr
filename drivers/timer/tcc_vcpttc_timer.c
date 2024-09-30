/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tcc_ttcvcp

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <soc.h>
#include <zephyr/drivers/timer/system_timer.h>
#include "xlnx_psttc_timer_priv.h"

#include <string.h>
#include "tcc_vcpttc_timer.h"

#ifdef CONFIG_TICKLESS_KERNEL
static uint32_t last_cycles;
#endif

static TIMERResource_t gTimerRes;

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = DT_IRQN(DT_INST(0, xlnx_ttcps));
#endif

static boolean gTimerInitialized = FALSE;

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

/* PLL Configuration Macro */
static void CLOCK_DevWritePll(uint32_t uiReg, uint32_t uiEn, uint32_t uiP, uint32_t uiM,
			      uint32_t uiS)
{
	const uint32_t uiTimeout = 4000000000UL;
	uint32_t uiDelay;
	uint32_t uiTryCnt;

	uiDelay = 0;
	uiTryCnt = 0;

	if (uiReg != 0UL) {
		if (uiEn != 0UL) {
			sys_write32(0UL | (1UL << (uint32_t)CLOCK_PLL_LOCKEN_SHIFT) |
					    (2UL << (uint32_t)CLOCK_PLL_CHGPUMP_SHIFT) |
					    (((uiS) & (uint32_t)CLOCK_PLL_S_MASK)
					     << (uint32_t)CLOCK_PLL_S_SHIFT) |
					    (((uiM) & (uint32_t)CLOCK_PLL_M_MASK)
					     << (uint32_t)CLOCK_PLL_M_SHIFT) |
					    (((uiP) & (uint32_t)CLOCK_PLL_P_MASK)
					     << (uint32_t)CLOCK_PLL_P_SHIFT),
				    uiReg);

			for (uiDelay = 100UL; uiDelay > 0UL; uiDelay--) {
				; /* if cpu clokc is 1HGz then loop 100. */
			}

			sys_write32(sys_read32(uiReg) |
					    (((uiEn)&1UL) << (uint32_t)CLOCK_PLL_EN_SHIFT),
				    uiReg);

			while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_PLL_LOCKST_SHIFT)) ==
			       0UL) {
				uiTryCnt++;

				if (uiTryCnt > uiTimeout) {
					break;
				}
			}
		} else {
			sys_write32((uint32_t)(sys_read32(uiReg)) &
					    (~(1UL << (uint32_t)CLOCK_PLL_EN_SHIFT)),
				    uiReg);
		}
	}
}

static void CLOCK_DevWritePclkCtrl(uint32_t uiReg, uint32_t uiMd, uint32_t uiEn, uint32_t uiSel,
				   uint32_t uiDiv, uint32_t uiType)
{
	if (uiType == (uint32_t)CLOCK_PCLKCTRL_TYPE_XXX) {
		sys_write32(sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_OUTEN_SHIFT),
			    uiReg);
		sys_write32(sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT), uiReg);
		sys_write32((sys_read32(uiReg) & ~((uint32_t)CLOCK_PCLKCTRL_SEL_MASK
						   << (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT)),
			    uiReg);
		sys_write32((sys_read32(uiReg) & ~((uint32_t)CLOCK_PCLKCTRL_DIV_XXX_MASK
						   << (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT)),
			    uiReg);

		sys_write32((sys_read32(uiReg) | ((uiDiv & (uint32_t)CLOCK_PCLKCTRL_DIV_XXX_MASK)
						  << (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT)),
			    uiReg);
		sys_write32((sys_read32(uiReg) | ((uiSel & (uint32_t)CLOCK_PCLKCTRL_SEL_MASK)
						  << (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT)),
			    uiReg);
		sys_write32(
			(sys_read32(uiReg) | ((uiEn & 1UL) << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT)),
			uiReg);
		sys_write32((sys_read32(uiReg) |
			     ((uiEn & 1UL) << (uint32_t)CLOCK_PCLKCTRL_OUTEN_SHIFT)),
			    uiReg);
	} else if (uiType == (uint32_t)CLOCK_PCLKCTRL_TYPE_YYY) {
		sys_write32(sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT), uiReg);
		sys_write32((sys_read32(uiReg) & ~((uint32_t)CLOCK_PCLKCTRL_DIV_YYY_MASK
						   << (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT)) |
				    ((uiDiv & (uint32_t)CLOCK_PCLKCTRL_DIV_YYY_MASK)
				     << (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT),
			    uiReg);
		sys_write32((sys_read32(uiReg) & ~((uint32_t)CLOCK_PCLKCTRL_SEL_MASK
						   << (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT)) |
				    ((uiSel & (uint32_t)CLOCK_PCLKCTRL_SEL_MASK)
				     << (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT),
			    uiReg);
		sys_write32((sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_MD_SHIFT)) |
				    ((uiMd & 1UL) << (uint32_t)CLOCK_PCLKCTRL_MD_SHIFT),
			    uiReg);
		sys_write32((sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT)) |
				    ((uiEn & 1UL) << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT),
			    uiReg);
	} else {
		; // empty statement
	}
}

static void CLOCK_DevWriteClkCtrl(uint32_t uiReg, uint32_t uiEn, uint32_t uiConf, uint32_t uiSel)
{
	const uint32_t uiTimeout = 4000000000;
	uint32_t uiCurConf;
	uint32_t uiTryCnt;

	uiCurConf = (sys_read32(uiReg) >> (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT) &
		    (uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK;

	if (uiConf >= uiCurConf) {
		sys_write32((sys_read32(uiReg) & (~((uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK
						    << (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT))) |
				    ((uiConf & (uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK)
				     << (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT),
			    uiReg);
		uiTryCnt = 0;

		while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_CLKCHG_SHIFT)) !=
		       0UL) {
			uiTryCnt++;

			if (uiTryCnt > uiTimeout) {
				break;
			}
		}

		sys_write32((sys_read32(uiReg) & (~((uint32_t)CLOCK_MCLKCTRL_SEL_MASK
						    << (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT))) |
				    ((uiSel & (uint32_t)CLOCK_MCLKCTRL_SEL_MASK)
				     << (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT),
			    uiReg);
		uiTryCnt = 0;

		while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_CLKCHG_SHIFT)) !=
		       0UL) {
			uiTryCnt++;

			if (uiTryCnt > uiTimeout) {
				break;
			}
		}
	} else {
		sys_write32((sys_read32(uiReg) & (~((uint32_t)CLOCK_MCLKCTRL_SEL_MASK
						    << (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT))) |
				    ((uiSel & (uint32_t)CLOCK_MCLKCTRL_SEL_MASK)
				     << (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT),
			    uiReg);
		uiTryCnt = 0;

		while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_CLKCHG_SHIFT)) !=
		       0UL) {
			uiTryCnt++;

			if (uiTryCnt > uiTimeout) {
				break;
			}
		}

		sys_write32((sys_read32(uiReg) & (~((uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK
						    << (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT))) |
				    ((uiConf & (uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK)
				     << (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT),
			    uiReg);
		uiTryCnt = 0;

		while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_CLKCHG_SHIFT)) !=
		       0UL) {
			uiTryCnt++;

			if (uiTryCnt > uiTimeout) {
				break;
			}
		}
	}

	if (uiEn != 0UL) {
		sys_write32((sys_read32(uiReg) & (~(1UL << (uint32_t)CLOCK_MCLKCTRL_EN_SHIFT))) |
				    ((uiEn & 1UL) << (uint32_t)CLOCK_MCLKCTRL_EN_SHIFT),
			    uiReg);
		uiTryCnt = 0;

		while ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_DIVSTS_SHIFT)) !=
		       0UL) {
			uiTryCnt++;

			if (uiTryCnt > uiTimeout) {
				break;
			}
		}
	}
}

static signed long CLOCK_DevFindPms(CLOCKPms_t *psPll, uint32_t uiSrcFreq)
{
	unsigned long long ullPll;
	unsigned long long ullSrc;
	unsigned long long ullFvco;
	unsigned long long ullSrchP;
	unsigned long long ullSrchM;
	unsigned long long ullTemp;
	unsigned long long ullSrchPll;

	uint32_t uiErr;
	uint32_t uiSrchErr;

	signed long iSrchS;

	if (psPll == NULL) {
		return 0;
	}

	if (psPll->uiFpll == 0UL) {
		psPll->uiEn = 0;
		return 0;
	}

	ullPll = (unsigned long long)psPll->uiFpll;
	ullSrc = (unsigned long long)uiSrcFreq;

	uiErr = 0xFFFFFFFFUL;
	uiSrchErr = 0xFFFFFFFFUL;

	for (iSrchS = (signed long)CLOCK_PLL_S_MIN; iSrchS <= (signed long)CLOCK_PLL_S_MAX;
	     iSrchS++) {
		ullFvco = ullPll << (unsigned long long)iSrchS;

		if ((ullFvco >= (unsigned long long)CLOCK_PLL_VCO_MIN) &&
		    (ullFvco <= (unsigned long long)CLOCK_PLL_VCO_MAX)) {
			for (ullSrchP = (unsigned long long)CLOCK_PLL_P_MIN;
			     ullSrchP <= (unsigned long long)CLOCK_PLL_P_MAX; ullSrchP++) {
				ullSrchM = ullFvco * ullSrchP;
				(void)FR_CoreDiv64To32(&(ullSrchM), (uint32_t)uiSrcFreq, NULL);

				if ((ullSrchM < (unsigned long long)CLOCK_PLL_M_MIN) ||
				    (ullSrchM > (unsigned long long)CLOCK_PLL_M_MAX)) {
					continue;
				}

				ullTemp = ullSrchM * ullSrc;
				(void)FR_CoreDiv64To32(&(ullTemp), (uint32_t)ullSrchP, NULL);
				ullSrchPll = (ullTemp >> (unsigned long long)iSrchS);

				if ((ullSrchPll < (unsigned long long)CLOCK_PLL_MIN_RATE) ||
				    (ullSrchPll > (unsigned long long)CLOCK_PLL_MAX_RATE)) {
					continue;
				}

				uiSrchErr =
					(uint32_t)((ullSrchPll > ullPll) ? (ullSrchPll - ullPll)
									 : (ullPll - ullSrchPll));

				if (uiSrchErr < uiErr) {
					uiErr = uiSrchErr;
					psPll->uiP = (uint32_t)ullSrchP;
					psPll->uiM = (uint32_t)ullSrchM;
					psPll->uiS = (uint32_t)iSrchS;
				}
			}
		}
	}

	if (uiErr == 0xFFFFFFFFUL) {
		return -1;
	}

	ullTemp = ullSrc * (unsigned long long)psPll->uiM;
	(void)FR_CoreDiv64To32(&(ullTemp), (uint32_t)(psPll->uiP), NULL);
	psPll->uiFpll = (uint32_t)(ullTemp >> psPll->uiS);
	psPll->uiEn = 1;

	return 0;
}

static signed long CLOCK_DevSetPllRate(uint32_t uiReg, uint32_t uiRate)
{
	CLOCKPms_t sPll;
	unsigned long long ullCalM;
	signed long iErr;

	iErr = 0;
	sPll.uiFpll = uiRate;

	if (CLOCK_DevFindPms(&sPll, CLOCK_XIN_CLK_RATE) != 0L) {
		// goto tcc_ckc_setpll2_failed;
		ullCalM =
			(unsigned long long)CLOCK_PLL_P_MIN * (unsigned long long)CLOCK_PLL_VCO_MIN;
		ullCalM += (unsigned long long)CLOCK_XIN_CLK_RATE;
		ullCalM /= (unsigned long long)CLOCK_XIN_CLK_RATE;
		CLOCK_DevWritePll(uiReg, 0UL, (uint32_t)CLOCK_PLL_P_MIN, (uint32_t)ullCalM,
				  (uint32_t)CLOCK_PLL_S_MIN);
		iErr = -1;
	} else {
		CLOCK_DevWritePll(uiReg, sPll.uiEn, sPll.uiP, sPll.uiM, sPll.uiS);
	}

	// return 0;

	// tcc_ckc_setpll2_failed:
	//   CLOCK_DevWritePll(uiReg, 0, CLOCK_PLL_P_MIN,
	//((CLOCK_PLL_P_MIN*CLOCK_PLL_VCO_MIN)+CLOCK_XIN_CLK_RATE)/CLOCK_XIN_CLK_RATE,
	// CLOCK_PLL_S_MIN);

	return iErr;
}

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

static uint32_t CLOCK_DevCalPclkDiv(const CLOCKPclkCtrl_t *psPclkCtrl, uint32_t *puiClkDiv,
				    const uint32_t uiSrcClk,
				    /*uint32_t                              uiDivMin,*/
				    uint32_t uiDivMax)
{
	uint32_t uiClkRate1;
	uint32_t uiClkRate2;
	uint32_t uiErr1;
	uint32_t uiErr2;

	if (psPclkCtrl == NULL) {
		return 0;
	}

	if (puiClkDiv == NULL) {
		return 0;
	}

	if (uiSrcClk <= psPclkCtrl->uiFreq) {
		*puiClkDiv = 1UL;
	} else {
		*puiClkDiv = uiSrcClk / psPclkCtrl->uiFreq;
	}

	if ((*puiClkDiv) > uiDivMax) {
		*puiClkDiv = uiDivMax;
	}

	uiClkRate1 = (uiSrcClk) / (*puiClkDiv);
	uiClkRate2 = (uiSrcClk) / (((*puiClkDiv) < uiDivMax) ? ((*puiClkDiv) + 1UL) : (*puiClkDiv));
	uiErr1 = (uiClkRate1 > psPclkCtrl->uiFreq) ? (uiClkRate1 - psPclkCtrl->uiFreq)
						   : (psPclkCtrl->uiFreq - uiClkRate1);
	uiErr2 = (uiClkRate2 > psPclkCtrl->uiFreq) ? (uiClkRate2 - psPclkCtrl->uiFreq)
						   : (psPclkCtrl->uiFreq - uiClkRate2);

	if (uiErr1 > uiErr2) {
		*puiClkDiv += 1UL;
	}

	return (uiErr1 < uiErr2) ? uiErr1 : uiErr2;
}

static signed long CLOCK_DevFindPclk(CLOCKPclkCtrl_t *psPclkCtrl, CLOCKPclkCtrlType_t eType)
{
	signed long iRet;
	signed long iLastIdx;
	signed long iIdx;
	uint32_t uiDivMax;
	uint32_t uiSrchSrc;
	uint32_t uiErrDiv;
	uint32_t uiMd;
	uint32_t uiDiv[CLOCK_SRC_MAX_NUM];
	uint32_t uiErr[CLOCK_SRC_MAX_NUM];
	uint32_t uiDivDiv;
	CLOCKPclkCtrlMode_t tPclkSel;

	uiDivDiv = CLOCK_PCLKCTRL_DIV_MIN;
	tPclkSel = CLOCK_PCLKCTRL_MODE_DIVIDER;
	iRet = 0;

	if (psPclkCtrl == NULL) {
		return -1;
	}

	psPclkCtrl->uiMd = (uint32_t)tPclkSel;
	uiDivMax = CLOCK_PCLKCTRL_DIV_XXX_MAX;

#if 1 // POOKY 20240924
	memset(uiDiv, 0x00U, sizeof(uiDiv));
#else
	(void)SAL_MemSet((void *)uiDiv, 0x00U, sizeof(uiDiv));
#endif
	uiSrchSrc = 0xFFFFFFFFUL;
	iLastIdx = (signed long)CLOCK_SRC_MAX_NUM - 1L;

	for (iIdx = iLastIdx; iIdx >= 0L; iIdx--) {
		if (stMicomClockSource[iIdx] == 0UL) {
			continue;
		}

		if ((stMicomClockSource[iIdx] >= (uint32_t)CLOCK_PCLKCTRL_MAX_FCKS) &&
		    (eType == CLOCK_PCLKCTRL_TYPE_XXX)) {
			continue;
		}

		/* divider mode */
		uiErrDiv = CLOCK_DevCalPclkDiv(
			psPclkCtrl, &uiDivDiv,
			stMicomClockSource[iIdx], /*CLOCK_PCLKCTRL_DIV_MIN+1, \*/
			uiDivMax + 1UL);

		uiErr[iIdx] = uiErrDiv;
		uiDiv[iIdx] = uiDivDiv;
		uiMd = (uint32_t)tPclkSel;

		if (uiSrchSrc == 0xFFFFFFFFUL) {
			uiSrchSrc = (uint32_t)iIdx;
			psPclkCtrl->uiMd = uiMd;
		} else {
			/* find similar clock */
			if (uiErr[iIdx] <= uiErr[uiSrchSrc]) {
				uiSrchSrc = (uint32_t)iIdx;
				psPclkCtrl->uiMd = uiMd;
			} else {
				;
			}
		}
	}

	switch (uiSrchSrc) {
	case (uint32_t)CLOCK_MPLL_0: {
		psPclkCtrl->uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_PLL0;
		break;
	}
	case (uint32_t)CLOCK_MPLL_1: {
		psPclkCtrl->uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_PLL1;
		break;
	}
	case (uint32_t)CLOCK_MPLL_DIV_0: {
		psPclkCtrl->uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_PLL0DIV;
		break;
	}

	case (uint32_t)CLOCK_MPLL_DIV_1: {
		psPclkCtrl->uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_PLL1DIV;
		break;
	}

	case (uint32_t)CLOCK_MPLL_XIN: {
		psPclkCtrl->uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_XIN;
		break;
	}

	default: {
		iRet = -1;
		break;
	}
	}

	if (iRet != 0) {
		return -1;
	}

	psPclkCtrl->uiDivVal = uiDiv[uiSrchSrc];

	if ((psPclkCtrl->uiDivVal >= ((uint32_t)CLOCK_PCLKCTRL_DIV_MIN + 1UL)) &&
	    (psPclkCtrl->uiDivVal <= (uiDivMax + 1UL))) {
		psPclkCtrl->uiDivVal -= 1UL;
	} else {
		return -1;
	}

	psPclkCtrl->uiFreq = stMicomClockSource[uiSrchSrc] / (psPclkCtrl->uiDivVal + 1UL);

	return 0;
}

signed long CLOCK_SetPllDiv(signed long iId, uint32_t uiPllDiv)
{
	uint32_t uiReg;
	CLOCKMpll_t uiMpllId;
	uint32_t uiOffSet;
	uint32_t uiRegVa;
	uint32_t uiRealPllDiv;

	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_CLKDIVC;
	uiMpllId = (CLOCKMpll_t)iId;

	uiRegVa = 0;
	uiRealPllDiv = ((uiPllDiv > 1UL) ? (uiPllDiv - 1UL) : (0UL));

	switch (uiMpllId) {
	case CLOCK_MPLL_0: {
		uiOffSet = (3UL - (uint32_t)iId) * 8UL;
		break;
	}

	case CLOCK_MPLL_1: {
		uiOffSet = (3UL - (uint32_t)iId) * 8UL;
		break;
	}

	case CLOCK_MPLL_XIN: {
		uiOffSet = 8UL;
		break;
	}

	default: {
		uiOffSet = 0;
		break;
	}
	}

	if (uiOffSet == 0UL) {
		return -1;
	}

	uiRegVa = (uint32_t)(sys_read32(uiReg)) & (~(((uint32_t)0xFFUL << uiOffSet)));
	sys_write32(uiRegVa, uiReg);

	if (uiRealPllDiv != 0UL) {
		uiRegVa |= (0x80UL | (uiRealPllDiv & 0x3FUL)) << uiOffSet;
	} else {
		uiRegVa |= (((uint32_t)0x01UL) << uiOffSet);
	}

	sys_write32(uiRegVa, uiReg);

	return 0;
}

signed long CLOCK_SetPllRate(signed long iId, uint32_t uiRate)
{
	uint32_t uiReg;
	CLOCKPll_t uiPllId;
	CLOCKMpclkCtrlSel_t tMpclkSel;
	signed long iIdx;

	uiReg = (uint32_t)0;
	uiPllId = (CLOCKPll_t)iId;
	iIdx = -1;

	if (uiPllId == CLOCK_PLL_MICOM_0) {
		uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PLL0PMS;
		tMpclkSel = CLOCK_MPCLKCTRL_SEL_PLL0;
	} else if (uiPllId == CLOCK_PLL_MICOM_1) {
		uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PLL1PMS;
		tMpclkSel = CLOCK_MPCLKCTRL_SEL_PLL1;
	} else {
		return -1;
	}

	iIdx = (signed long)tMpclkSel;
	(void)CLOCK_DevSetPllRate(uiReg, uiRate);
	stMicomClockSource[iIdx] = CLOCK_DevGetPllRate(uiReg);

	return 0;
}

static signed long CLOCK_DevFindClkCtrl(CLOCKClkCtrl_t *CLKCTRL)
{
	signed long iRet;
	uint32_t uiIdx;
	uint32_t uiDivTable[CLOCK_SRC_MAX_NUM];
	uint32_t uiErr[CLOCK_SRC_MAX_NUM];
	uint32_t uiSrchSrc;
	uint32_t uiClkRate;
	// signed long uiXinFreq = CLKCTRL->uiEn ? (CLOCK_XIN_CLK_RATE/2) : CLOCK_XIN_CLK_RATE;
	uint32_t uiXinFreq;

	iRet = 0;
	uiXinFreq = 0;

	if (CLKCTRL == NULL) {
		return -1;
	}

	if (CLKCTRL->uiEn != 0UL) {
		uiXinFreq = ((uint32_t)CLOCK_XIN_CLK_RATE / 2UL);
	} else {
		uiXinFreq = (uint32_t)CLOCK_XIN_CLK_RATE;
	}

	uiSrchSrc = 0xFFFFFFFFUL;

	if (CLKCTRL->uiFreq <= uiXinFreq) {
		CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_XIN;
		CLKCTRL->uiFreq = uiXinFreq;
		// CLKCTRL->uiConf = CLKCTRL->uiEn ? 1 : 0;

		if (CLKCTRL->uiEn != 0UL) {
			CLKCTRL->uiConf = 1;
		} else {
			CLKCTRL->uiConf = 0;
		}
	} else {
#if 1 // POOKY 20240924
		memset(uiDivTable, 0x00U, sizeof(uiDivTable));
#else
		(void)SAL_MemSet((void *)uiDivTable, 0x00U, sizeof(uiDivTable));
#endif

		for (uiIdx = 0UL; uiIdx < (uint32_t)CLOCK_SRC_MAX_NUM; uiIdx++) {
			if (stMicomClockSource[uiIdx] == 0UL) {
				continue;
			}

			if (CLKCTRL->uiEn != 0UL) {
				uiDivTable[uiIdx] =
					(stMicomClockSource[uiIdx] + (CLKCTRL->uiFreq - 1UL)) /
					CLKCTRL->uiFreq;
				if (uiDivTable[uiIdx] >
				    ((uint32_t)CLOCK_MCLKCTRL_CONFIG_MAX + 1UL)) {
					uiDivTable[uiIdx] =
						(uint32_t)CLOCK_MCLKCTRL_CONFIG_MAX + 1UL;
				} else if (uiDivTable[uiIdx] <
					   ((uint32_t)CLOCK_MCLKCTRL_CONFIG_MIN + 1UL)) {
					uiDivTable[uiIdx] =
						(uint32_t)CLOCK_MCLKCTRL_CONFIG_MIN + 1UL;
				} else {
					; // empty statement
				}

				uiClkRate = stMicomClockSource[uiIdx] / uiDivTable[uiIdx];
			} else {
				uiClkRate = stMicomClockSource[uiIdx];
			}

			if (CLKCTRL->uiFreq < uiClkRate) {
				continue;
			}

			uiErr[uiIdx] = CLKCTRL->uiFreq - uiClkRate;

			if (uiSrchSrc == 0xFFFFFFFFUL) {
				uiSrchSrc = uiIdx;
			} else {
				/* find similar clock */
				if (uiErr[uiIdx] < uiErr[uiSrchSrc]) {
					uiSrchSrc = uiIdx;
				}
				/* find even division vlaue */
				else if (uiErr[uiIdx] == uiErr[uiSrchSrc]) {
					if ((uiDivTable[uiIdx] % 2UL) == 0UL) {
						uiSrchSrc = uiIdx;
					}
				} else {
					uiSrchSrc = 0xFFFFFFFFUL;
				}
			}

			if (uiErr[uiSrchSrc] == 0UL) {
				break;
			}
		}

		if (uiSrchSrc == 0xFFFFFFFFUL) {
			return -1;
		}

		switch (uiSrchSrc) {
		case (uint32_t)CLOCK_MPLL_0: {
			CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_PLL0;
			break;
		}

		case (uint32_t)CLOCK_MPLL_1: {
			CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_PLL1;
			break;
		}

		case (uint32_t)CLOCK_MPLL_DIV_0: {
			CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_PLL0DIV;
			break;
		}

		case (uint32_t)CLOCK_MPLL_DIV_1: {
			CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_PLL1DIV;
			break;
		}

		case (uint32_t)CLOCK_MPLL_XIN: {
			CLKCTRL->uiSel = (uint32_t)CLOCK_MCLKCTRL_SEL_XIN;
			break;
		}

		default: {
			iRet = -1;
			break;
		}
		}

		if (iRet != 0L) {
			return -1;
		}

		if (CLKCTRL->uiEn != 0UL) {
			if (uiDivTable[uiSrchSrc] > ((uint32_t)CLOCK_MCLKCTRL_CONFIG_MAX + 1UL)) {
				uiDivTable[uiSrchSrc] = (uint32_t)CLOCK_MCLKCTRL_CONFIG_MAX + 1UL;
			} else if (uiDivTable[uiSrchSrc] <= (uint32_t)CLOCK_MCLKCTRL_CONFIG_MIN) {
				uiDivTable[uiSrchSrc] = (uint32_t)CLOCK_MCLKCTRL_CONFIG_MIN + 1UL;
			} else {
				; // else
			}

			CLKCTRL->uiFreq = stMicomClockSource[uiSrchSrc] / uiDivTable[uiSrchSrc];
			CLKCTRL->uiConf = uiDivTable[uiSrchSrc] - 1UL;
		} else {
			CLKCTRL->uiFreq = stMicomClockSource[uiSrchSrc];
			CLKCTRL->uiConf = 0;
		}
	}

	return 0;
}

signed long CLOCK_SetClkCtrlRate(signed long iId, uint32_t uiRate)
{
	uint32_t uiReg;
	CLOCKClkCtrl_t sClkCtrl;

	uiReg = (uint32_t)((uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_CLKCTRL +
			   ((uint32_t)iId * 4UL));

	// sClkCtrl.uiEn = (sys_read32(uiReg) & (1<<CLOCK_MCLKCTRL_EN_SHIFT)) ? 1 : 0;
	if ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_MCLKCTRL_EN_SHIFT)) != 0UL) {
		sClkCtrl.uiEn = 1;
	} else {
		sClkCtrl.uiEn = 0;
	}

	sClkCtrl.uiFreq = uiRate;

	if (CLOCK_DevFindClkCtrl(&sClkCtrl) != 0L) {
		return -1;
	}

	CLOCK_DevWriteClkCtrl(uiReg, sClkCtrl.uiEn, sClkCtrl.uiConf, sClkCtrl.uiSel);

	return 0;
}

uint32_t CLOCK_GetClkCtrlRate(signed long iId)
{
	uint32_t uiReg;
	CLOCKClkCtrl_t sClkCtrl;
	CLOCKMclkCtrlSel_t tMclkSel;
	uint32_t uiRegVa;
	uint32_t uiSrcFreq;
	uint32_t uiRet;

	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_CLKCTRL + ((uint32_t)iId * 4UL);
	uiRegVa = 0;
	uiRet = 0;
	uiRegVa = sys_read32(uiReg);

	sClkCtrl.uiSel = (uiRegVa & ((uint32_t)CLOCK_MCLKCTRL_SEL_MASK
				     << (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT)) >>
			 (uint32_t)CLOCK_MCLKCTRL_SEL_SHIFT;
	tMclkSel = (CLOCKMclkCtrlSel_t)sClkCtrl.uiSel;

	switch (tMclkSel) {
	case CLOCK_MCLKCTRL_SEL_XIN: {
		uiSrcFreq = (uint32_t)CLOCK_XIN_CLK_RATE;
		break;
	}

	case CLOCK_MCLKCTRL_SEL_PLL0: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_0);
		break;
	}

	case CLOCK_MCLKCTRL_SEL_PLL1: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_1);
		break;
	}

	case CLOCK_MCLKCTRL_SEL_PLL0DIV: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_0) /
			    (CLOCK_DevGetPllDiv((signed long)CLOCK_PLL_MICOM_0) + 1UL);
		break;
	}

	case CLOCK_MCLKCTRL_SEL_PLL1DIV: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_1) /
			    (CLOCK_DevGetPllDiv((signed long)CLOCK_PLL_MICOM_1) + 1UL);
		break;
	}

	default: {
		uiSrcFreq = 0UL;
		break;
	}
	}

	if (uiSrcFreq > 0UL) {
		sClkCtrl.uiConf = (uiRegVa & ((uint32_t)CLOCK_MCLKCTRL_CONFIG_MASK
					      << (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT)) >>
				  (uint32_t)CLOCK_MCLKCTRL_CONFIG_SHIFT;

		sClkCtrl.uiFreq = uiSrcFreq / (sClkCtrl.uiConf + 1UL);
		uiRet = (uint32_t)sClkCtrl.uiFreq;
	}

	return uiRet;
}

signed long CLOCK_IsPeriEnabled(signed long iId)
{
	CLOCKPeri_t uiPeriOffset;
	uint32_t uiReg;

	uiPeriOffset = CLOCK_PERI_SFMC;
	uiReg = ((uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PCLKCTRL) +
		(((uint32_t)iId - (uint32_t)uiPeriOffset) * 4UL);

	return ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT)) != 0UL) ? 1L : 0L;
}

signed long CLOCK_EnablePeri(signed long iId)
{
	CLOCKPeri_t uiPeriOffset;
	uint32_t uiReg;

	uiPeriOffset = CLOCK_PERI_SFMC;
	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PCLKCTRL +
		(((uint32_t)iId - (uint32_t)uiPeriOffset) * 4UL);
	sys_write32((sys_read32(uiReg) | (1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT)), uiReg);

	return 0;
}

signed long CLOCK_DisablePeri(signed long iId)
{
	CLOCKPeri_t uiPeriOffset;
	uint32_t uiReg;

	uiPeriOffset = CLOCK_PERI_SFMC;
	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PCLKCTRL +
		(((uint32_t)iId - (uint32_t)uiPeriOffset) * 4UL);

	sys_write32(sys_read32(uiReg) & ~(1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT), uiReg);

	return 0;
}

uint32_t CLOCK_GetPeriRate(signed long iId)
{
	CLOCKPeri_t uiPeriOffset;
	uint32_t uiReg;
	uint32_t uiRegVa;
	uint32_t uiSrcFreq;
	uint32_t uiRet;
	CLOCKPclkCtrl_t sPclkCtrl;
	CLOCKPclkctrlSel_t uiPclkSel;

	uiPeriOffset = CLOCK_PERI_SFMC;
	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PCLKCTRL +
		(((uint32_t)iId - (uint32_t)uiPeriOffset) * 4UL);
	uiRegVa = sys_read32(uiReg);
	uiRet = 0;
	sPclkCtrl.uiSel = (uiRegVa & ((uint32_t)CLOCK_PCLKCTRL_SEL_MASK
				      << (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT)) >>
			  (uint32_t)CLOCK_PCLKCTRL_SEL_SHIFT;

	uiPclkSel = (CLOCKPclkctrlSel_t)(sPclkCtrl.uiSel);

	switch (uiPclkSel) {
	case CLOCK_PCLKCTRL_SEL_PLL0: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_0);
		break;
	}

	case CLOCK_PCLKCTRL_SEL_PLL1: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_1);
		break;
	}

	case CLOCK_PCLKCTRL_SEL_PLL0DIV: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_0) /
			    (CLOCK_DevGetPllDiv((signed long)CLOCK_PLL_MICOM_0) + 1UL);
		break;
	}

	case CLOCK_PCLKCTRL_SEL_PLL1DIV: {
		uiSrcFreq = CLOCK_GetPllRate((signed long)CLOCK_PLL_MICOM_1) /
			    (CLOCK_DevGetPllDiv((signed long)CLOCK_PLL_MICOM_1) + 1UL);
		break;
	}

	case CLOCK_PCLKCTRL_SEL_XIN: {
		uiSrcFreq = (uint32_t)CLOCK_XIN_CLK_RATE;
		break;
	}

	default: {
		uiSrcFreq = 0UL;
		break;
	}
	}

	if (uiSrcFreq > 0UL) {
		sPclkCtrl.uiFreq = 0;
		sPclkCtrl.uiDivVal = (uiRegVa & ((uint32_t)CLOCK_PCLKCTRL_DIV_XXX_MASK
						 << (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT)) >>
				     (uint32_t)CLOCK_PCLKCTRL_DIV_SHIFT;
		sPclkCtrl.uiFreq = uiSrcFreq / (sPclkCtrl.uiDivVal + 1UL);
		uiRet = (uint32_t)sPclkCtrl.uiFreq;
	}

	return uiRet;
}

signed long CLOCK_SetPeriRate(signed long iId, uint32_t uiRate)
{
	CLOCKPeri_t uiPeriOffset;
	uint32_t uiReg;
	signed long iErr;
	CLOCKPclkCtrl_t sPclkCtrl;

	uiPeriOffset = CLOCK_PERI_SFMC;
	uiReg = (uint32_t)CLOCK_BASE_ADDR + (uint32_t)CLOCK_MCKC_PCLKCTRL +
		(((uint32_t)iId - (uint32_t)uiPeriOffset) * 4UL);
	iErr = 0;
	sPclkCtrl.uiFreq = uiRate;
	sPclkCtrl.uiPeriName = (uint32_t)iId;
	sPclkCtrl.uiDivVal = 0;
	sPclkCtrl.uiMd = (uint32_t)CLOCK_PCLKCTRL_MODE_DIVIDER;
	sPclkCtrl.uiSel = (uint32_t)CLOCK_MPCLKCTRL_SEL_XIN;

	if (CLOCK_DevFindPclk(&sPclkCtrl, CLOCK_PCLKCTRL_TYPE_XXX) != 0L) {
		// goto tcc_ckc_setperi_failed;
		CLOCK_DevWritePclkCtrl(uiReg, (uint32_t)CLOCK_PCLKCTRL_MODE_DIVIDER,
				       (uint32_t)FALSE, (uint32_t)CLOCK_MPCLKCTRL_SEL_XIN, 1UL,
				       (uint32_t)CLOCK_PCLKCTRL_TYPE_XXX);
		iErr = -1;
	} else {
		// sPclkCtrl.uiEn = (sys_read32(uiReg) & (1<<CLOCK_PCLKCTRL_EN_SHIFT)) ? 1 : 0;
		if ((sys_read32(uiReg) & (1UL << (uint32_t)CLOCK_PCLKCTRL_EN_SHIFT)) != 0UL) {
			sPclkCtrl.uiEn = 1;
		} else {
			sPclkCtrl.uiEn = 0;
		}

		CLOCK_DevWritePclkCtrl(uiReg, sPclkCtrl.uiMd, sPclkCtrl.uiEn, sPclkCtrl.uiSel,
				       sPclkCtrl.uiDivVal, (uint32_t)CLOCK_PCLKCTRL_TYPE_XXX);
	}

	return iErr;
}

signed long CLOCK_IsIobusPwdn(signed long iId)
{
	uint32_t uiReg;
	signed long iRest;
	CLOCKIobus_t tIobus;

	tIobus = (CLOCKIobus_t)iId;

	if ((signed long)tIobus < (32L * 1L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK0;
	} else if ((signed long)tIobus < (32L * 2L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK1;
	} else if ((signed long)tIobus < (32L * 3L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK2;
	} else {
		return -1;
	}

	iRest = (signed long)tIobus % 32L;

	return ((sys_read32(uiReg) & ((uint32_t)1UL << (uint32_t)iRest)) != 0UL) ? 0L : 1L;
}

signed long CLOCK_EnableIobus(signed long iId, boolean bEn)
{
	signed long iRet;

	if (bEn == TRUE) {
		if (CLOCK_SetIobusPwdn(iId, FALSE) == 0L) {
			iRet = CLOCK_SetSwReset(iId, FALSE);
		} else {
			iRet = -1;
		}
	} else {
		if (CLOCK_SetSwReset(iId, TRUE) == 0L) {
			iRet = CLOCK_SetIobusPwdn(iId, TRUE);
		} else {
			iRet = -1;
		}
	}

	return iRet;
}

signed long CLOCK_SetIobusPwdn(signed long iId, boolean bEn)
{
	uint32_t uiReg;
	signed long iRest;
	CLOCKIobus_t tIobus;

	tIobus = (CLOCKIobus_t)iId;

	if ((signed long)tIobus < (32L * 1L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK0;
	} else if ((signed long)tIobus < (32L * 2L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK1;
	} else if ((signed long)tIobus < (32L * 3L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLK2;
	} else {
		return -1;
	}

	iRest = (signed long)tIobus % 32;

	if (bEn == TRUE) {
		sys_write32(sys_read32(uiReg) & ~((uint32_t)1UL << (uint32_t)iRest), uiReg);
	} else {
		sys_write32(sys_read32(uiReg) | ((uint32_t)1UL << (uint32_t)iRest), uiReg);
	}

	return 0;
}

signed long CLOCK_SetSwReset(signed long iId, boolean bReset)
{
	uint32_t uiReg;
	signed long iRest;
	CLOCKIobus_t tIobus;

	tIobus = (CLOCKIobus_t)iId;

	if ((signed long)tIobus < (32L * 1L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLKSWR0;
	} else if ((signed long)tIobus < (32L * 2L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLKSWR1;
	} else if ((signed long)tIobus < (32L * 3L)) {
		uiReg = (uint32_t)MCU_BSP_SUBSYS_BASE + (uint32_t)CLOCK_MCKC_HCLKSWR2;
	} else {
		return -1;
	}

	iRest = (signed long)tIobus % 32;

	if (bReset == TRUE) {
		sys_write32(sys_read32(uiReg) & ~((uint32_t)1UL << (uint32_t)iRest), uiReg);
	} else {
		sys_write32(sys_read32(uiReg) | ((uint32_t)1UL << (uint32_t)iRest), uiReg);
	}

	return 0;
}

static uint32_t read_count(void)
{
	/* Read current counter value */
	return sys_read32(TIMER_BASE_ADDR + TMR_MAIN_CNT);
}

static void update_match(uint32_t cycles, uint32_t match)
{
	uint32_t delta = match - cycles;

	/* Ensure that the match value meets the minimum timing requirements */
	if (delta < CYCLES_NEXT_MIN) {
		match += CYCLES_NEXT_MIN - delta;
	}

	/* Write counter match value for interrupt generation */
	sys_write32(match, TIMER_BASE_ADDR + TMR_CMP_VALUE0);
}

SALRetCode_t TIMER_InterruptClear(TIMERChannel_t uiChannel);
static void ttc_isr(TIMERChannel_t uiChannel, const void *arg)
{
	uint32_t cycles;
	uint32_t ticks;

	ARG_UNUSED(arg);

	/* Acknowledge interrupt */
	TIMER_InterruptClear(uiChannel);

	/* Read counter value */
	cycles = read_count();

#ifdef CONFIG_TICKLESS_KERNEL
	/* Calculate the number of ticks since last announcement  */
	ticks = (cycles - last_cycles) / CYCLES_PER_TICK;

	/* Update last cycles count */
	last_cycles = cycles;
#else
	/* Update counter match value for the next interrupt */
	update_match(cycles, cycles + CYCLES_PER_TICK);

	/* Advance tick count by 1 */
	ticks = 1;
#endif

	/* Announce to the kernel*/
	sys_clock_announce(ticks);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
#ifdef CONFIG_TICKLESS_KERNEL
	uint32_t cycles;
	uint32_t next_cycles;

	/* Read counter value */
	cycles = read_count();

	/* Calculate timeout counter value */
	if (ticks == K_TICKS_FOREVER) {
		next_cycles = cycles + CYCLES_NEXT_MAX;
	} else {
		next_cycles = cycles + ((uint32_t)ticks * CYCLES_PER_TICK);
	}

	/* Set match value for the next interrupt */
	update_match(cycles, next_cycles);
#endif
}

uint32_t sys_clock_elapsed(void)
{
#ifdef CONFIG_TICKLESS_KERNEL
	uint32_t cycles;

	/* Read counter value */
	cycles = read_count();

	/* Return the number of ticks since last announcement */
	return (cycles - last_cycles) / CYCLES_PER_TICK;
#else
	/* Always return 0 for tickful operation */
	return 0;
#endif
}

uint32_t sys_clock_cycle_get_32(void)
{
	/* Return the current counter value */
	return read_count();
}

SALRetCode_t TIMER_InterruptClear(TIMERChannel_t uiChannel)
{
	uint32_t reg;
	uint32_t clr_ctl;

	reg = TIMER_BASE_ADDR + TMR_IRQ_CTRL;
	// Deviation Record - CERT INT36-C, CERT-C Integers
	clr_ctl = sys_read32(reg);

	if ((clr_ctl & TMR_IRQ_CLR_CTRL_WRITE) != 0U) {
		// Deviation Record - CERT INT36-C, CERT-C Integers
		sys_write32(clr_ctl | TMR_IRQ_MASK_ALL, reg);
	} else // TMR_IRQ_CLR_CTRL_READ
	{
		// Deviation Record - CERT INT36-C, CERT-C Integers
		sys_read32(reg);
	}

	return SAL_RET_SUCCESS;
}

#if 0
static void TIMER_Handler(void *pArgs)
{
	TIMERResource_t *timer = NULL_PTR;
	uint32_t reg;

	memcpy(&timer, (const void *)&pArgs, sizeof(TIMERResource_t *));

	if (timer != NULL_PTR) {
		reg = TIMER_BASE_ADDR + TMR_IRQ_CTRL;

		// Deviation Record - CERT INT36-C, CERT-C Integers
		if (((sys_read32(reg) & TMR_IRQ_CTRL_IRQ_ALLEN) != 0UL) &&
		    (timer->rtUsed == TRUE)) {
			(void)TIMER_InterruptClear(timer->rtChannel);

			if (timer->rtHandler != NULL_PTR) {
				(void)timer->rtHandler(timer->rtChannel, timer->rtArgs);
			}
		}
	}
}
#endif

static void TIMER_SetEnableCoreReg(const TIMERConfig_t *pCfg, const uint32_t uiCmpVal0,
				   const uint32_t uiCmpVal1, uint32_t uiCfgVal,
				   const uint32_t uiIrqVal)
{
	uint32_t mainval = 0x0UL;
	uint32_t tmpval = 0x0UL;
	uint32_t reg = TIMER_BASE_ADDR;
	uint32_t rate_factor = (TMR_CLK_RATE / 1000UL) / ((TMR_PRESCALE + 1UL) * 1000UL);

	mainval = (pCfg->ctMainValueUsec == 0UL) ? 0xFFFFFFFFUL
						 : ((pCfg->ctMainValueUsec * rate_factor) - 1UL);

	// reset main cnt load value
	//  Deviation Record - CERT INT36-C, CERT-C Integers
	sys_write32(mainval, (uint32_t)(reg + TMR_MAIN_CNT_LVD));
	sys_write32(uiCmpVal0, (uint32_t)(reg + TMR_CMP_VALUE0));
	sys_write32(uiCmpVal1, (uint32_t)(reg + TMR_CMP_VALUE1));

	uiCfgVal |= (TMR_PRESCALE | TMR_OP_EN_CFG_CNT_EN |
		     ((uint32_t)pCfg->ctStartMode << TMR_OP_EN_CFG_LDZERO_OFFSET));

	if (pCfg->ctOpMode == TIMER_OP_ONESHOT) {
		uiCfgVal |= TMR_OP_EN_CFG_OPMODE_ONE_SHOT;
	}

	// Deviation Record - CERT INT36-C, CERT-C Integers
	tmpval = sys_read32((uint32_t)(reg + TMR_IRQ_CTRL));

	// Deviation Record - CERT INT36-C, CERT-C Integers
	sys_write32(uiCfgVal, (uint32_t)(reg + TMR_OP_EN_CFG));
	sys_write32((tmpval | uiIrqVal), (uint32_t)(reg + TMR_IRQ_CTRL));
}

static SALRetCode_t TIMER_EnableComp0(const TIMERConfig_t *pCfg)
{
	uint32_t tmpval = 0x0UL;
	uint32_t rate_factor;
	uint32_t mainval = 0x0UL;
	uint32_t cmpval0 = 0x0UL;
	uint32_t cmpval1 = 0x0UL;
	uint32_t reg_cfgval = 0x0UL;
	uint32_t reg_irqval = TMR_IRQ_CTRL_IRQ_EN2;
	SALRetCode_t ret = SAL_RET_SUCCESS;

	rate_factor = (TMR_CLK_RATE / 1000UL) / ((TMR_PRESCALE + 1UL) * 1000UL);

	// CERT-C Integers (CERT INT30-C) : Ensure that unsigned integer operations do not wrap
	if ((SAL_MAX_INT_VAL / rate_factor) < pCfg->ctCmp0ValueUsec) {
		ret = SAL_RET_FAILED;
	} else {
		mainval = (pCfg->ctMainValueUsec == 0UL)
				  ? 0xFFFFFFFFUL
				  : ((pCfg->ctMainValueUsec * rate_factor) - 1UL);
		tmpval = (pCfg->ctCmp0ValueUsec * rate_factor) - 1UL;

		if ((pCfg->ctStartMode == TIMER_START_MAINCNT) &&
		    (((0xFFFFFFFFUL - tmpval) == 0xFFFFFFFFUL) ||
		     (mainval > (0xFFFFFFFFUL - tmpval)))) {
			ret = SAL_RET_FAILED;
		} else {
			cmpval0 = (pCfg->ctStartMode == TIMER_START_ZERO) ? tmpval
									  : (mainval + tmpval);
			reg_cfgval = TMR_OP_EN_CFG_LDM0_ON;
			reg_irqval |= TMR_IRQ_CTRL_IRQ_EN0;

			TIMER_SetEnableCoreReg(pCfg, cmpval0, cmpval1, reg_cfgval, reg_irqval);
		}
	}

	return ret;
}

static SALRetCode_t TIMER_EnableComp1(const TIMERConfig_t *pCfg)
{
	uint32_t tmpval = 0x0UL;
	uint32_t rate_factor;
	uint32_t mainval;
	uint32_t cmpval0 = 0x0UL;
	uint32_t cmpval1 = 0x0UL;
	uint32_t reg_cfgval = 0x0UL;
	uint32_t reg_irqval = TMR_IRQ_CTRL_IRQ_EN2;
	SALRetCode_t ret = SAL_RET_SUCCESS;

	rate_factor = (TMR_CLK_RATE / 1000UL) / ((TMR_PRESCALE + 1UL) * 1000UL);

	// CERT-C Integers (CERT INT30-C) : Ensure that unsigned integer operations do not wrap
	if ((SAL_MAX_INT_VAL / rate_factor) < pCfg->ctCmp1ValueUsec) {
		ret = SAL_RET_FAILED;
	} else {
		mainval = (pCfg->ctMainValueUsec == 0UL)
				  ? 0xFFFFFFFFUL
				  : ((pCfg->ctMainValueUsec * rate_factor) - 1UL);
		tmpval = (pCfg->ctCmp1ValueUsec * rate_factor) - 1UL;

		if ((pCfg->ctStartMode == TIMER_START_MAINCNT) &&
		    (((0xFFFFFFFFUL - tmpval) == 0xFFFFFFFFUL) ||
		     (mainval > (0xFFFFFFFFUL - tmpval)))) {
			ret = SAL_RET_FAILED;
		} else {
			cmpval1 = (pCfg->ctStartMode == TIMER_START_ZERO) ? tmpval
									  : (mainval + tmpval);
			reg_cfgval = TMR_OP_EN_CFG_LDM1_ON;
			reg_irqval |= TMR_IRQ_CTRL_IRQ_EN1;

			TIMER_SetEnableCoreReg(pCfg, cmpval0, cmpval1, reg_cfgval, reg_irqval);
		}
	}

	return ret;
}

static SALRetCode_t TIMER_EnableSmallComp(const TIMERConfig_t *pCfg)
{
	uint32_t rate_factor;
	uint32_t tmpval0 = 0x0UL;
	uint32_t tmpval1 = 0x0UL;
	uint32_t mainval = 0x0UL;
	uint32_t cmpval0 = 0x0UL;
	uint32_t cmpval1 = 0x0UL;
	uint32_t reg_cfgval = 0x0UL;
	uint32_t reg_irqval = TMR_IRQ_CTRL_IRQ_EN2;
	SALRetCode_t ret = SAL_RET_SUCCESS;

	rate_factor = (TMR_CLK_RATE / 1000UL) / ((TMR_PRESCALE + 1UL) * 1000UL);
	mainval = (pCfg->ctMainValueUsec == 0UL) ? 0xFFFFFFFFUL
						 : ((pCfg->ctMainValueUsec * rate_factor) - 1UL);

	// CERT-C Integers (CERT INT30-C) : Ensure that unsigned integer operations do not wrap
	if ((SAL_MAX_INT_VAL / rate_factor) < pCfg->ctCmp0ValueUsec) {
		ret = SAL_RET_FAILED;
	} else if ((SAL_MAX_INT_VAL / rate_factor) < pCfg->ctCmp1ValueUsec) {
		ret = SAL_RET_FAILED;
	} else {
		tmpval0 = (pCfg->ctCmp0ValueUsec * rate_factor) - 1UL;
		tmpval1 = (pCfg->ctCmp1ValueUsec * rate_factor) - 1UL;

		if (pCfg->ctStartMode == TIMER_START_MAINCNT) {
			// Less value is selected
			if (tmpval0 <= tmpval1) {
				// CERT-C Integers (CERT INT30-C) : Ensure that unsigned integer
				// operations do not wrap
				if ((SAL_MAX_INT_VAL - mainval) <= tmpval0) {
					ret = SAL_RET_FAILED;
				} else {
					cmpval0 = mainval + tmpval0;
					cmpval1 = SAL_MAX_INT_VAL;
				}
			} else {
				if ((SAL_MAX_INT_VAL - mainval) <= tmpval1) {
					ret = SAL_RET_FAILED;
				} else {
					cmpval0 = SAL_MAX_INT_VAL;
					cmpval1 = mainval + tmpval1;
				}
			}
		} else {
			cmpval0 = tmpval0;
			cmpval1 = tmpval1;
		}

		if (ret == SAL_RET_SUCCESS) {
			reg_cfgval = (TMR_OP_EN_CFG_LDM0_ON | TMR_OP_EN_CFG_LDM1_ON);
			reg_irqval |= (TMR_IRQ_CTRL_IRQ_EN0 | TMR_IRQ_CTRL_IRQ_EN1);

			TIMER_SetEnableCoreReg(pCfg, cmpval0, cmpval1, reg_cfgval, reg_irqval);
		}
	}

	return ret;
}

static SALRetCode_t TIMER_EnableMode(const TIMERConfig_t *pCfg)
{
	SALRetCode_t ret = SAL_RET_SUCCESS;

	switch (pCfg->ctCounterMode) {
	case TIMER_COUNTER_COMP0: {
		ret = TIMER_EnableComp0(pCfg);

		break;
	}

	case TIMER_COUNTER_COMP1: {
		ret = TIMER_EnableComp1(pCfg);

		break;
	}

	case TIMER_COUNTER_SMALL_COMP: {
		ret = TIMER_EnableSmallComp(pCfg);

		break;
	}

	default: // TIMER_COUNTER_MAIN
	{
		TIMER_SetEnableCoreReg(pCfg, 0x0UL, 0x0UL, 0x0UL, TMR_IRQ_CTRL_IRQ_EN2);

		break;
	}
	}

	return ret;
}

SALRetCode_t TIMER_EnableWithCfg(const TIMERConfig_t *pCfg)
{
	SALRetCode_t ret = SAL_RET_SUCCESS;

	// MISRA 14.7 : A function shall have a single point of exit at the end of the function
	if (gTimerInitialized == FALSE) {
		ret = SAL_RET_FAILED;
	} else if (pCfg == NULL_PTR) {
		ret = SAL_RET_FAILED;
	} else if (TIMER_CH_MAX <= pCfg->ctChannel) {
		ret = SAL_RET_FAILED;
	} else {
		ret = TIMER_EnableMode(pCfg);

		if (ret == SAL_RET_SUCCESS) {
			gTimerRes.rtUsed = TRUE;
			gTimerRes.rtHandler = pCfg->fnHandler;
			gTimerRes.rtArgs = pCfg->pArgs;

#if 0 // 20240924 POOKY
			if (pCfg->ctChannel != TIMER_CH_0) // timer 0 specially used by os timer
			{
				(void)GIC_IntVectSet(
					(uint32_t)GIC_TIMER_0 + (uint32_t)pCfg->ctChannel,
					GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_LEVEL_HIGH,
					(GICIsrFunc)&TIMER_Handler, (void *)&gTimerRes);
			} // (ch != TIMER_CH_0)

			// TMR_D("Channel (%d) is enabled\n", (uint32_t)pCfg->ctChannel);
#endif
		} // (ret == SAL_RET_SUCCESS)
	}

	return ret;
}

SALRetCode_t TIMER_EnableWithMode(TIMERChannel_t uiChannel, uint32_t uiUSec, TIMEROpMode_t uiOpMode,
				  TIMERHandler fnHandler, void *pArgs)
{
	TIMERConfig_t cfg;

	cfg.ctChannel = uiChannel;
	cfg.ctStartMode = TIMER_START_ZERO;
	cfg.ctOpMode = uiOpMode;
	cfg.ctCounterMode = TIMER_COUNTER_COMP0;
	cfg.ctMainValueUsec = 0;
	cfg.ctCmp0ValueUsec = uiUSec;
	cfg.ctCmp1ValueUsec = 0;
	cfg.fnHandler = fnHandler;
	cfg.pArgs = pArgs;

	return TIMER_EnableWithCfg(&cfg);
}

SALRetCode_t TIMER_Enable(uint32_t uiChannel, uint32_t uiUSec, TIMERHandler fnHandler, void *pArgs)
{
	return TIMER_EnableWithMode(uiChannel, uiUSec, TIMER_OP_FREERUN, fnHandler, pArgs);
}

static int sys_clock_driver_init(void)
{
#if 1 // POOKY 20240923
	uint32_t reg_val;
	SALRetCode_t ret;
	uint32_t timer_channel;

	//	CLOCK_Init();

#ifdef CONFIG_TICKLESS_KERNEL
	/* Initialise internal states */
	last_cycles = 0;
#endif

	ret = SAL_RET_SUCCESS;

	switch (TIMER_BASE_ADDR) {
	case 0xA0F2A000:
		timer_channel = 0;
		break;
	case 0xA0F2A100:
		timer_channel = 1;
		break;
	case 0xA0F2A200:
		timer_channel = 2;
		break;
	case 0xA0F2A300:
		timer_channel = 3;
		break;
	case 0xA0F2A400:
		timer_channel = 4;
		break;
	case 0xA0F2A500:
		timer_channel = 5;
		break;
	case 0xA0F2A600:
		timer_channel = 6;
		break;
	case 0xA0F2A700:
		timer_channel = 7;
		break;
	case 0xA0F2A800:
		timer_channel = 8;
		break;
	case 0xA0F2A900:
		timer_channel = 9;
		break;
	default:
		return SAL_RET_FAILED;
	}

	gTimerRes.rtChannel = (TIMERChannel_t)timer_channel;
	gTimerRes.rtUsed = FALSE;
	gTimerRes.rtHandler = NULL_PTR;
	gTimerRes.rtArgs = NULL_PTR;

	/* Stop timer */

	/* Initialise timer registers */
	/* Reset counter value */
	// Deviation Record - CERT INT36-C, CERT-C Integers
	sys_write32(0x7FFFU, (uint32_t)(TIMER_BASE_ADDR + TMR_OP_EN_CFG));
	sys_write32(0x0U, (uint32_t)(TIMER_BASE_ADDR + TMR_MAIN_CNT_LVD));
	sys_write32(0x0U, (uint32_t)(TIMER_BASE_ADDR + TMR_CMP_VALUE0));
	sys_write32(0x0U, (uint32_t)(TIMER_BASE_ADDR + TMR_CMP_VALUE1));

	reg_val = TMR_IRQ_CLR_CTRL_WRITE |
		  TMR_IRQ_MASK_ALL; // = TMR_IRQ_CLR_CTRL_READ | TMR_IRQ_MASK_ALL;

	sys_write32(reg_val, (uint32_t)(TIMER_BASE_ADDR + TMR_IRQ_CTRL));

	gTimerInitialized = TRUE;

	/* Set match mode */

	/* Set initial timeout */
	reg_val = IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? CYCLES_NEXT_MAX : CYCLES_PER_TICK;

	/* Connect timer interrupt */
	IRQ_CONNECT(TIMER_IRQ, 0, ttc_isr, 0, 0);

	/* Enable timer interrupt */
	irq_enable(TIMER_IRQ);

	/* Start timer */
	(void)TIMER_Enable(timer_channel, reg_val, (TIMERHandler)&ttc_isr, NULL_PTR);

	return ret;

#else // POOKY 20240923
	uint32_t reg_val;

	/* Stop timer */
	sys_write32(XTTCPS_CNT_CNTRL_DIS_MASK, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

#ifdef CONFIG_TICKLESS_KERNEL
	/* Initialise internal states */
	last_cycles = 0;
#endif

	/* Initialise timer registers */
	sys_write32(XTTCPS_CNT_CNTRL_RESET_VALUE, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_CLK_CNTRL_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_INTERVAL_VAL_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_0_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_1_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_2_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);
	sys_write32(XTTCPS_IXR_ALL_MASK, TIMER_BASE_ADDR + XTTCPS_ISR_OFFSET);

	/* Reset counter value */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val |= XTTCPS_CNT_CNTRL_RST_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	/* Set match mode */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val |= XTTCPS_CNT_CNTRL_MATCH_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	/* Set initial timeout */
	reg_val = IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? CYCLES_NEXT_MAX : CYCLES_PER_TICK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_MATCH_0_OFFSET);

	/* Connect timer interrupt */
	IRQ_CONNECT(TIMER_IRQ, 0, ttc_isr, 0, 0);
	irq_enable(TIMER_IRQ);

	/* Enable timer interrupt */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);
	reg_val |= XTTCPS_IXR_MATCH_0_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);

	/* Start timer */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val &= (~XTTCPS_CNT_CNTRL_DIS_MASK);
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	return 0;
#endif // POOKY 20240923
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
