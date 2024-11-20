/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NOTE: This driver implements the GIC400 interfaces.
 */

#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sw_isr_table.h>
//#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/sys/barrier.h>

#define DT_DRV_COMPAT tcc_tic

static GICIntFuncPtr_t GICIsrTable[GIC_INT_SRC_CNT];

static uint32_t gCPU_SR = 0;

static void core_mb(void)
{
	CPU_DSB();
}

static void core_critical_enter(void)
{
	gCPU_SR = 0;
	gCPU_SR = CPU_SR_Save();
}

static void core_critical_exit(void)
{
	CPU_SR_Restore(gCPU_SR);
}

static void GIC_IntPrioSet_internal(uint32_t uiIntId, uint32_t uiPrio)
{
	uint32_t uiRegOffset;
	uint32_t uiRegBitField;
	uint32_t uiGICD_IPRIORITYRn;

	uiRegOffset = 0;
	uiRegBitField = 0;
	uiGICD_IPRIORITYRn = 0;

	if ((uiPrio < GIC_PRIORITY_NO_MEAN) && (uiIntId < GIC_INT_SRC_CNT)) {
		uiRegOffset = (uiIntId >> 2u);
		uiRegBitField = (uiIntId & 0x03u);

		uiGICD_IPRIORITYRn = GIC_DIST->dIPRIORITYRn[uiRegOffset];
		uiGICD_IPRIORITYRn =
			(uint32_t)(uiGICD_IPRIORITYRn & ~((uint32_t)0xFFu << (uiRegBitField * 8u)));
		uiGICD_IPRIORITYRn = (uint32_t)(uiGICD_IPRIORITYRn |
						(((uiPrio << 4) & 0xF0u) << (uiRegBitField * 8u)));

		GIC_DIST->dIPRIORITYRn[uiRegOffset] = uiGICD_IPRIORITYRn;
	}
}

static void GIC_IntConfigSet(uint32_t uiIntId, uint8_t ucIntType)
{
	uint32_t uiRegOffset;
	uint32_t uiRegMask;
	uint32_t uiGICD_ICFGRn;

	uiRegOffset = 0;
	uiRegMask = 0;
	uiGICD_ICFGRn = 0;

	if (uiIntId < GIC_INT_SRC_CNT) {
		uiRegOffset = (uiIntId >> 4u);
		uiRegMask = (uint32_t)((uint32_t)0x2u << ((uiIntId & 0xfu) * 2u));
		uiGICD_ICFGRn = GIC_DIST->dICFGRn[uiRegOffset];

		if (((ucIntType & (uint8_t)GIC_INT_TYPE_LEVEL_HIGH) ==
		     (uint8_t)GIC_INT_TYPE_LEVEL_HIGH) ||
		    ((ucIntType & (uint8_t)GIC_INT_TYPE_LEVEL_LOW) ==
		     (uint8_t)GIC_INT_TYPE_LEVEL_LOW)) {
			uiGICD_ICFGRn = (uint32_t)(uiGICD_ICFGRn & ~uiRegMask);
		} else {
			uiGICD_ICFGRn = (uint32_t)(uiGICD_ICFGRn | uiRegMask);
		}

		GIC_DIST->dICFGRn[uiRegOffset] = uiGICD_ICFGRn;
	}
}

void GIC_IntVectSet(uint32_t uiIntId, uint32_t uiPrio, uint8_t ucIntType, GICIsrFunc fnIntFunc,
		    void *pIntArg)
{
	uint32_t uiRevIntId;

	uiRevIntId = 0;

	if ((uiPrio > GIC_PRIORITY_NO_MEAN) || (uiIntId >= GIC_INT_SRC_CNT)) {
		return;
	}

	core_critical_enter(); /* Prevent partially configured interrupts. */

	(void)GIC_IntPrioSet_internal(uiIntId, uiPrio);
	(void)GIC_IntConfigSet(uiIntId, ucIntType);

	GICIsrTable[uiIntId].ifpFunc = fnIntFunc;
	GICIsrTable[uiIntId].ifpArg = pIntArg;
	GICIsrTable[uiIntId].ifpIsBothEdge = 0;

	if ((uiIntId >= (uint32_t)GIC_EINT_START_INT) &&
	    (uiIntId <= (uint32_t)GIC_EINT_END_INT) /* Set reversed external interrupt */
	    && (ucIntType == (uint8_t)GIC_INT_TYPE_EDGE_BOTH)) { /* for supporting both edge. */

		uiRevIntId = (uiIntId + GIC_EINT_NUM); /* add offset of IRQn */

		(void)GIC_IntPrioSet_internal(uiRevIntId, uiPrio);
		(void)GIC_IntConfigSet(uiRevIntId, ucIntType);

		GICIsrTable[uiRevIntId].ifpFunc = fnIntFunc;
		GICIsrTable[uiRevIntId].ifpArg = pIntArg;
		GICIsrTable[uiIntId].ifpIsBothEdge = (1U);
	}

	core_critical_exit();
}

SALRetCode_t GIC_IntSrcEn ( uint32_t                              uiIntId)
{
    z_tic_irq_enable(uiIntId);

    return SAL_RET_SUCCESS;
}

void GIC_IntSrcDis(uint32_t uiIntId)
{
	z_tic_irq_disable(uiIntId);
}

unsigned int z_tic_irq_get_active(void)
{
	uint32_t int_id;

	int_id = GIC_CPU->cIAR;

	return (int_id);
}

void z_tic_irq_enable(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5u); /* Calculate the register offset. */
		uiBit = (irq & 0x1Fu);     /* Mask bit ID.     */

		GIC_DIST->dISENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);

		if (GICIsrTable[irq].ifpIsBothEdge == (1UL)) {
			uiRegOffset = ((irq + 10UL) >> 5UL); /* Calculate the register offset. */
			uiBit = ((irq + 10UL) & 0x1FUL);     /* Mask bit ID.     */

			GIC_DIST->dISENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);
		}

		core_mb();
	}
}

void z_tic_irq_disable(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5UL); /* Calculate the register offset. */
		uiBit = (irq & 0x1FUL);     /* Mask bit ID.     */

		GIC_DIST->dICENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);

		if (GICIsrTable[irq].ifpIsBothEdge == (1UL)) {
			uiRegOffset = ((irq + 10UL) >> 5UL); /* Calculate the register offset. */
			uiBit = ((irq + 10UL) & 0x1FUL);     /* Mask bit ID.     */

			GIC_DIST->dICENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);
		}
	}
}

bool z_tic_irq_is_enabled(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;
	uint32_t enabler;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5u); /* Calculate the register offset. */
		uiBit = (irq & 0x1Fu);     /* Mask bit ID.     */

		enabler = GIC_DIST->dISENABLERn[uiRegOffset];

		return (enabler & (1 << uiBit)) != 0;
	}

	return false;
}

bool z_tic_irq_is_pending(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;
	uint32_t enabler;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5u); /* Calculate the register offset. */
		uiBit = (irq & 0x1Fu);     /* Mask bit ID.     */

		enabler = GIC_DIST->dISPENDRn[uiRegOffset];

		return (enabler & (1 << uiBit)) != 0;
	}

	return false;
}

void z_tic_irq_set_pending(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5u); /* Calculate the register offset. */
		uiBit = (irq & 0x1Fu);     /* Mask bit ID.     */

		GIC_DIST->dISPENDRn[uiRegOffset] = ((uint32_t)1UL << uiBit);

		if (GICIsrTable[irq].ifpIsBothEdge == (1UL)) {
			uiRegOffset = ((irq + 10UL) >> 5UL); /* Calculate the register offset. */
			uiBit = ((irq + 10UL) & 0x1FUL);     /* Mask bit ID.     */

			GIC_DIST->dISPENDRn[uiRegOffset] = ((uint32_t)1UL << uiBit);
		}

		core_mb();
	}
}

void z_tic_irq_clear_pending(unsigned int irq)
{
	uint32_t uiRegOffset;
	uint32_t uiBit;

	uiRegOffset = 0;
	uiBit = 0;

	if (irq < GIC_INT_SRC_CNT) {
		uiRegOffset = (irq >> 5UL); /* Calculate the register offset. */
		uiBit = (irq & 0x1FUL);     /* Mask bit ID.     */

		GIC_DIST->dICPENDRn[uiRegOffset] = ((uint32_t)1UL << uiBit);

		if (GICIsrTable[irq].ifpIsBothEdge == (1UL)) {
			uiRegOffset = ((irq + 10UL) >> 5UL); /* Calculate the register offset. */
			uiBit = ((irq + 10UL) & 0x1FUL);     /* Mask bit ID.     */

			GIC_DIST->dICPENDRn[uiRegOffset] = ((uint32_t)1UL << uiBit);
		}
	}
}

static void tic_irq_priority_set_internal(uint32_t uiIntId, uint32_t uiPrio)
{
	uint32_t uiRegOffset;
	uint32_t uiRegBitField;
	uint32_t uiGICD_IPRIORITYRn;

	uiRegOffset = 0;
	uiRegBitField = 0;
	uiGICD_IPRIORITYRn = 0;

	if ((uiPrio < GIC_PRIORITY_NO_MEAN) && (uiIntId < GIC_INT_SRC_CNT)) {
		uiRegOffset = (uiIntId >> 2u);
		uiRegBitField = (uiIntId & 0x03u);

		uiGICD_IPRIORITYRn = GIC_DIST->dIPRIORITYRn[uiRegOffset];
		uiGICD_IPRIORITYRn =
			(uint32_t)(uiGICD_IPRIORITYRn & ~((uint32_t)0xFFu << (uiRegBitField * 8u)));
		uiGICD_IPRIORITYRn = (uint32_t)(uiGICD_IPRIORITYRn |
						(((uiPrio << 4) & 0xF0u) << (uiRegBitField * 8u)));

		GIC_DIST->dIPRIORITYRn[uiRegOffset] = uiGICD_IPRIORITYRn;
	}
}

void z_tic_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	core_critical_enter();
	tic_irq_priority_set_internal(irq, prio);
	core_critical_exit();
}

void z_tic_irq_eoi(unsigned int irq)
{
	GIC_CPU->cEOIR = irq;
}

void tic_raise_sgi(unsigned int sgi_id, uint64_t target_aff, uint16_t target_list)
{
	uint32_t uiTargetListFilter;
	uint32_t uiCPUTargetList;
	uint32_t uiNASTT;

	uiTargetListFilter = GIC_SGI_TO_TARGETLIST;
	uiCPUTargetList = 0x1UL; /* bitfiled 0 : cpu #0, bitfield n : cpu #n, n : 0 ~ 7          */
	uiNASTT = 0UL;           /* 0 : group 0 , 1: group 1                                     */

	if (sgi_id <= 15UL) {
		GIC_DIST->dSGIR = (uint32_t)((uiTargetListFilter & 0x3UL) << 24) |
				  ((uiCPUTargetList & 0xffUL) << 16) | ((uiNASTT & 0x1UL) << 15) |
				  (sgi_id & 0xfUL);
	}
}

void z_tic_irq_init(void)
{
	unsigned long uiRegOffset;

	uiRegOffset = 0;

	/* Global GIC disable -> enable. */
	GIC_DIST->dCTRL &= (unsigned long)(~ARM_BIT_GIC_DIST_ICDDCR_EN);
	GIC_DIST->dCTRL |= (unsigned long)ARM_BIT_GIC_DIST_ICDDCR_EN;

	for (; uiRegOffset <= ((unsigned long)(GIC_INT_SRC_CNT - 1UL) / 4UL); uiRegOffset++) {
		GIC_DIST->dIPRIORITYRn[uiRegOffset] = 0xA0A0A0A0UL;
	}

	GIC_CPU->cPMR = 0xFFUL;
	GIC_CPU->cCTLR |= GIC_CPUIF_CTRL_ENABLEGRP0;

	core_mb();
}

/**
 * @brief Initialize the GIC device driver
 */
int z_tic_init(const struct device *dev)
{
	z_tic_irq_init();

	return 0;
}

DEVICE_DT_INST_DEFINE(0, z_tic_init, NULL, NULL, NULL, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,
		      NULL);
