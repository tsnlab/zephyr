/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <cmsis_core.h>
#include <string.h>
#include "soc.h"

unsigned int z_soc_irq_get_active(void)
{
	return z_tic_irq_get_active();
}

void z_soc_irq_eoi(unsigned int irq)
{
	z_tic_irq_eoi(irq);
}

void z_soc_irq_init(void)
{
	z_tic_irq_init();
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	/* Configure interrupt type and priority */
	z_tic_irq_priority_set(irq, prio, flags);
}

void z_soc_irq_enable(unsigned int irq)
{
	/* Enable interrupt */
	z_tic_irq_enable(irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	/* Disable interrupt */
	z_tic_irq_disable(irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	/* Check if interrupt is enabled */
	return z_tic_irq_is_enabled(irq);
}

void CLOCK_Init(void);
//void PMU_Init(void);

void BSP_EnableSYSPower(void);
void soc_reset_hook(void)
{
	/*
	 * Use normal exception vectors address range (0x0-0x1C).
	 */
	unsigned int sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_V_Msk;
	__set_SCTLR(sctlr);
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 */
void soc_early_init_hook(void)
{
	CLOCK_Init();
	//	GIC_Init();

	//PMU_Init();

	BSP_EnableSYSPower();
}

/*
 * sal_internal
 */

SALRetCode_t SAL_CoreMB (void)
{
    CPU_DSB();

    return SAL_RET_SUCCESS;
}

SALRetCode_t SAL_MemSet(void *pMem, unsigned char ucValue, uint32_t uiSize)
{
	SALRetCode_t retVal = SAL_RET_SUCCESS;

	if (pMem != NULL_PTR) {
		(void)memset(pMem, (int)ucValue, (size_t)uiSize); // QAC-Not use return value
	} else {
		retVal = SAL_RET_FAILED;
	}

	return retVal;
}

SALRetCode_t SAL_MemCopy ( void * pDest, const void * pSrc, uint32_t uiSize) 
{
    SALRetCode_t retVal = SAL_RET_SUCCESS;

    if ((pDest != NULL_PTR) && (pSrc != NULL_PTR)) {
        (void)memcpy(pDest, pSrc, (size_t)uiSize); //QAC-Not use return value
    } else {
		retVal = SAL_RET_FAILED;
    }

    return retVal;
}


SALRetCode_t SAL_CoreDiv64To32(unsigned long long *pullDividend, uint32_t uiDivisor,
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

void BSP_TestLEDOn(void)
{
	/* TEST_LED_BLINK */
	(void)GPIO_Config(TEST_LED_BLINK, (unsigned long)(GPIO_FUNC(0U) | VCP_GPIO_OUTPUT));
	(void)GPIO_Set(TEST_LED_BLINK, 1UL);
}

