/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tcc_ttcvcp

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#if 0
#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#endif
#include <zephyr/dt-bindings/interrupt-controller/tcc-tic.h>
#include <zephyr/drivers/interrupt_controller/intc_tic.h>
#include <soc.h>
//#include <zephyr/drivers/timer/system_timer.h>

//#include <string.h>
#include "tcc_vcpttc_timer.h"

#if 0
#define TIMER_INDEX     CONFIG_XLNX_PSTTC_TIMER_INDEX

#define TIMER_IRQ       DT_INST_IRQN(0)
#define TIMER_BASE_ADDR     DT_INST_REG_ADDR(0)
#define TIMER_CLOCK_FREQUECY    DT_INST_PROP(0, clock_frequency)

#define TICKS_PER_SEC       CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CYCLES_PER_SEC      TIMER_CLOCK_FREQUECY
#define CYCLES_PER_TICK     (CYCLES_PER_SEC / TICKS_PER_SEC)
#endif

#define SAL_OS_TIMER_ID                 (0UL)     /* HW Timer Resource for OS sheduling */
#define configTICK_RATE_HZ              ( ( uint32_t ) 1000UL )

#define TIMER_IRQ_NUM   DT_INST_IRQN(0)
#define TIMER_IRQ_PRIO  DT_INST_IRQ(0, priority)
#define TIMER_IRQ_FLAGS DT_INST_IRQ(0, flags)

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = DT_IRQN(DT_INST(0, tcc_ttcvcp));
#endif

static struct k_spinlock lock;
static uint32_t last_cycle;

static boolean gTimerInitialized = FALSE;

static TIMERResource_t                  gTimerRes[TIMER_CH_MAX];

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
static void ttc_timer_compare_isr(const void *arg)
{
#if 1

    ARG_UNUSED(arg);

//    k_spinlock_key_t key = k_spin_lock(&lock);

    (void)TIMER_InterruptClear(SAL_OS_TIMER_ID);
    uint32_t curr_cycle = read_count();
    //uint32_t curr_cycle = sys_read32(TIMER_BASE_ADDR + TMR_MAIN_CNT);
    //uint32_t delta_cycles = curr_cycle - last_cycle;
    uint32_t delta_ticks = (curr_cycle - last_cycle) / CYCLES_PER_TICK;


    last_cycle = curr_cycle;

//    k_spin_unlock(&lock, key);

    sys_clock_announce(delta_ticks);
#else
    TIMERResource_t *   timer = NULL_PTR;
    uint32_t              reg;
    uint32_t delta_ticks = 1;

    (void)SAL_MemCopy(&timer, (const void *)&arg, sizeof(TIMERResource_t *));

    if (timer != NULL_PTR)
    {
        reg = MCU_BSP_TIMER_BASE + ((uint32_t)timer->rtChannel * 0x100UL) + TMR_IRQ_CTRL;

        // Deviation Record - CERT INT36-C, CERT-C Integers
        if (((sys_read32(reg) & TMR_IRQ_CTRL_IRQ_ALLEN) != 0UL) && (timer->rtUsed == TRUE))
        {
            sys_clock_announce(delta_ticks);
        }
    }
#endif
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
#if 1
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
	uint32_t cycles;

	/* Read counter value */
	cycles = read_count();

	/* Return the number of ticks since last announcement */
	return (cycles - last_cycle) / CYCLES_PER_TICK;
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

static void TIMER_Handler(void *pArgs)
{
	TIMERResource_t *timer = NULL_PTR;
	uint32_t reg;

    SAL_MemCopy(&timer, (const void *)&pArgs, sizeof(TIMERResource_t *));

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
			gTimerRes[pCfg->ctChannel].rtUsed = TRUE;
			gTimerRes[pCfg->ctChannel].rtHandler = pCfg->fnHandler;
			gTimerRes[pCfg->ctChannel].rtArgs = pCfg->pArgs;

            if (pCfg->ctChannel != SAL_OS_TIMER_ID) {
                (void)GIC_IntVectSet(
                    (uint32_t)GIC_TIMER_0 + (uint32_t)pCfg->ctChannel,
                    GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_LEVEL_HIGH,
                    (GICIsrFunc)&TIMER_Handler, (void *)&gTimerRes[pCfg->ctChannel]);
            }
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

SALRetCode_t TIMER_Init (void)
{
    uint32_t          reg;
    uint32_t          resIndex;
    uint32_t          reg_val;
    SALRetCode_t    ret;

    ret = SAL_RET_SUCCESS;

    if(gTimerInitialized == TRUE) {
        return ret;
    }

    for (resIndex = 0 ; resIndex < (uint32_t)TIMER_CH_MAX ; resIndex++)
    {
        reg = MCU_BSP_TIMER_BASE + (resIndex * 0x100UL);

        gTimerRes[resIndex].rtChannel  = (TIMERChannel_t)resIndex;
        gTimerRes[resIndex].rtUsed     = FALSE;
        gTimerRes[resIndex].rtHandler  = NULL_PTR;
        gTimerRes[resIndex].rtArgs     = NULL_PTR;

        // Deviation Record - CERT INT36-C, CERT-C Integers
        sys_write32(0x7FFFU, (uint32_t)(reg + TMR_OP_EN_CFG));
        sys_write32(0x0U,    (uint32_t)(reg + TMR_MAIN_CNT_LVD));
        sys_write32(0x0U,    (uint32_t)(reg + TMR_CMP_VALUE0));
        sys_write32(0x0U,    (uint32_t)(reg + TMR_CMP_VALUE1));

        reg_val = TMR_IRQ_CLR_CTRL_WRITE | TMR_IRQ_MASK_ALL;    // = TMR_IRQ_CLR_CTRL_READ | TMR_IRQ_MASK_ALL;

        sys_write32(reg_val, (uint32_t)(reg + TMR_IRQ_CTRL));
    }

    gTimerInitialized = TRUE;

    return ret;
}


static int sys_clock_driver_init(void)
{
#if 1
	uint32_t timer_channel;
    uint32_t uiTickToUSec;

    TIMER_Init ();

	timer_channel = ((TIMER_BASE_ADDR - 0xA0F2A000) / 0x100);
    if(timer_channel >= TIMER_CH_MAX) {
		return SAL_RET_FAILED;
    }

    if(timer_channel != SAL_OS_TIMER_ID) {
		return SAL_RET_FAILED;
    }

    last_cycle = 0;
    //IRQ_CONNECT(TIMER_IRQ_NUM, TIMER_IRQ_PRIO, ttc_timer_compare_isr, NULL, TIMER_IRQ_FLAGS);
    IRQ_CONNECT(TIMER_IRQ_NUM, TIMER_IRQ_PRIO, ttc_timer_compare_isr, NULL, 0);

    (void)GIC_IntVectSet(TIMER_IRQ,
               IRQ_DEFAULT_PRIORITY,
               GIC_INT_TYPE_LEVEL_HIGH,
               (GICIsrFunc)ttc_timer_compare_isr,
               NULL);

    /* Start the timer */
    //uiTickToUSec = (uint32_t)(((uint32_t)1000U*(uint32_t)1000U)/configTICK_RATE_HZ);
    uiTickToUSec = (uint32_t)((uint32_t)30000U*(uint32_t)1000U);

    (void)TIMER_Enable((TIMERChannel_t)timer_channel, uiTickToUSec, 0, 0);

    // irq_enable(TIMER_IRQ_NUM);

    return 0;

#else
	SALRetCode_t ret;
	uint32_t timer_channel;
    uint32_t uiTickToUSec;

    TIMER_Init ();
#ifdef CONFIG_TICKLESS_KERNEL
	/* Initialise internal states */
	last_cycles = 0;
#endif

	ret = SAL_RET_SUCCESS;

	timer_channel = ((TIMER_BASE_ADDR - 0xA0F2A000) / 0x100);
    if(timer_channel >= TIMER_CH_MAX) {
		return SAL_RET_FAILED;
    }

    if(timer_channel == SAL_OS_TIMER_ID) {
        (void)GIC_IntVectSet(TIMER_IRQ,
                   IRQ_DEFAULT_PRIORITY,
                   GIC_INT_TYPE_LEVEL_HIGH,
                   (GICIsrFunc)ttc_timer_compare_isr,
                   NULL);
    } else {
	    return SAL_RET_SUCCESS;
    }

    (void)GIC_IntSrcEn(TIMER_IRQ);
    uiTickToUSec = (uint32_t)(((uint32_t)1000U*(uint32_t)1000U)/configTICK_RATE_HZ);

    (void)TIMER_Enable((TIMERChannel_t)timer_channel, uiTickToUSec, 0, 0);

    SAL_CoreMB();

	return ret;
#endif
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
