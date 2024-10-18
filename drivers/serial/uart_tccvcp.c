/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tcc_tccvcp_uart

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/types.h>
#include <soc.h>

#include <zephyr/init.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#include <string.h>

#include "uart_tccvcp.h"

static uart_status_t uart[UART_CH_MAX];

static uint8_t uart_buff0[2][UART_BUFF_SIZE];
static uint8_t uart_buff1[2][UART_BUFF_SIZE];
static uint8_t uart_buff2[2][UART_BUFF_SIZE];
static uint8_t uart_buff3[2][UART_BUFF_SIZE];
static uint8_t uart_buff4[2][UART_BUFF_SIZE];
static uint8_t uart_buff5[2][UART_BUFF_SIZE];

static uint8_t *uart_buff[UART_CH_MAX][2] = {
    {uart_buff0[0], uart_buff0[1]}, {uart_buff1[0], uart_buff1[1]},
    {uart_buff2[0], uart_buff2[1]}, {uart_buff3[0], uart_buff3[1]},
    {uart_buff4[0], uart_buff4[1]}, {uart_buff5[0], uart_buff5[1]}};

static GICIntFuncPtr_t GICIsrTable[GIC_INT_SRC_CNT]; /* Interrupt vector table. */

/* static uint32_t gCPU_SR = 0; */

int mfio_ch_cfg_flag[3] = {
    0,
};

static SALRetCode_t FR_CoreCriticalEnter(void)
{
    /* gCPU_SR = 0; */
    __asm__("    MRS     R0, CPSR");
    __asm__("    CPSID   IF");
    __asm__("    DSB");
    /* __asm__("    BX      LR"); */
    /* gCPU_SR = CPU_SR_Save(); */

    return SAL_RET_SUCCESS;
}

static SALRetCode_t FR_CoreCriticalExit(void)
{
    __asm__("    DSB");
    __asm__("    MSR     CPSR_c, R0");
    /* __asm__("    BX      LR"); */
    /* CPU_SR_Restore(gCPU_SR); */

    return SAL_RET_SUCCESS;
}

static SALRetCode_t GIC_IntPrioSet_internal(uint32_t uiIntId, uint32_t uiPrio)
{
    uint32_t uiRegOffset;
    uint32_t uiRegBitField;
    uint32_t uiGICD_IPRIORITYRn;
    SALRetCode_t ucRet;

    uiRegOffset = 0;
    uiRegBitField = 0;
    uiGICD_IPRIORITYRn = 0;
    ucRet = (SALRetCode_t)SAL_RET_FAILED;

    if ((uiPrio < GIC_PRIORITY_NO_MEAN) && (uiIntId < GIC_INT_SRC_CNT)) {
        uiRegOffset = (uiIntId >> 2u);
        uiRegBitField = (uiIntId & 0x03u);

        uiGICD_IPRIORITYRn = GIC_DIST->dIPRIORITYRn[uiRegOffset];
        uiGICD_IPRIORITYRn =
            (uint32_t)(uiGICD_IPRIORITYRn & ~((uint32_t)0xFFu << (uiRegBitField * 8u)));
        uiGICD_IPRIORITYRn = (uint32_t)(uiGICD_IPRIORITYRn |
                        (((uiPrio << 4) & 0xF0u) << (uiRegBitField * 8u)));

        GIC_DIST->dIPRIORITYRn[uiRegOffset] = uiGICD_IPRIORITYRn;
        ucRet = (SALRetCode_t)SAL_RET_SUCCESS;
    }

    return ucRet;
}

static SALRetCode_t GIC_IntConfigSet(uint32_t uiIntId, uint8_t ucIntType)
{
    uint32_t uiRegOffset;
    uint32_t uiRegMask;
    uint32_t uiGICD_ICFGRn;
    SALRetCode_t ucRet;

    uiRegOffset = 0;
    uiRegMask = 0;
    uiGICD_ICFGRn = 0;
    ucRet = (SALRetCode_t)SAL_RET_FAILED;

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
        ucRet = (SALRetCode_t)SAL_RET_SUCCESS;
    }

    return ucRet;
}

SALRetCode_t GIC_IntVectSet(uint32_t uiIntId, uint32_t uiPrio, uint8_t ucIntType,
                GICIsrFunc fnIntFunc, void *pIntArg)
{
    uint32_t uiRevIntId;
    SALRetCode_t ucRet;

    uiRevIntId = 0;
    ucRet = (SALRetCode_t)SAL_RET_SUCCESS;

    if ((uiPrio > GIC_PRIORITY_NO_MEAN) || (uiIntId >= GIC_INT_SRC_CNT)) {
        ucRet = (SALRetCode_t)SAL_RET_FAILED;
    } else {
        (void)FR_CoreCriticalEnter(); /* Prevent partially configured interrupts. */

        (void)GIC_IntPrioSet_internal(uiIntId, uiPrio);
        (void)GIC_IntConfigSet(uiIntId, ucIntType);

        GICIsrTable[uiIntId].ifpFunc = fnIntFunc;
        GICIsrTable[uiIntId].ifpArg = pIntArg;
        GICIsrTable[uiIntId].ifpIsBothEdge = 0;

        if ((uiIntId >= (uint32_t)GIC_EINT_START_INT) &&
            (uiIntId <= (uint32_t)GIC_EINT_END_INT) /* Set reversed external interrupt */
            && (ucIntType ==
            (uint8_t)GIC_INT_TYPE_EDGE_BOTH)) { /* for supporting both edge. */

            uiRevIntId = (uiIntId + GIC_EINT_NUM); /* add offset of IRQn */

            (void)GIC_IntPrioSet_internal(uiRevIntId, uiPrio);
            (void)GIC_IntConfigSet(uiRevIntId, ucIntType);

            GICIsrTable[uiRevIntId].ifpFunc = fnIntFunc;
            GICIsrTable[uiRevIntId].ifpArg = pIntArg;
            GICIsrTable[uiIntId].ifpIsBothEdge = (1U);
        }

        (void)FR_CoreCriticalExit();
    }

    return ucRet;
}

SALRetCode_t GIC_IntSrcDis(uint32_t uiIntId)
{
    SALRetCode_t ucRet;
    uint32_t uiRegOffset;
    uint32_t uiBit;

    ucRet = (SALRetCode_t)SAL_RET_FAILED;
    uiRegOffset = 0;
    uiBit = 0;

    if (uiIntId < GIC_INT_SRC_CNT) {
        uiRegOffset = (uiIntId >> 5UL); /* Calculate the register offset. */
        uiBit = (uiIntId & 0x1FUL);     /* Mask bit ID.     */

        GIC_DIST->dICENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);

        if (GICIsrTable[uiIntId].ifpIsBothEdge == (1UL)) {
            uiRegOffset =
                ((uiIntId + 10UL) >> 5UL);   /* Calculate the register offset.   */
            uiBit = ((uiIntId + 10UL) & 0x1FUL); /* Mask bit ID. */

            GIC_DIST->dICENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);
        }
        ucRet = (SALRetCode_t)SAL_RET_SUCCESS;
    }

    return ucRet;
}

static void UART_RegWrite(uint8_t ucCh, uint32_t uiAddr, uint32_t uiSetValue)
{
    uint32_t uiBaseAddr;
    uint32_t uiRegAddr;

    if (uart[ucCh].sBase == 0UL) {
        uart[ucCh].sBase = UART_GET_BASE(ucCh);
    }

    uiBaseAddr = uart[ucCh].sBase & 0xAFFFFFFFU;
    uiAddr &= 0xFFFFU;
    uiRegAddr = uiBaseAddr + uiAddr;
    sys_write32(uiSetValue, uiRegAddr);
}

static void UART_DisableInterrupt(uint8_t ucCh)
{
    if (ucCh < UART_CH_MAX) {
        (void)GIC_IntVectSet((uint32_t)GIC_UART0 + ucCh, GIC_PRIORITY_NO_MEAN,
                     GIC_INT_TYPE_LEVEL_HIGH, NULL_PTR, NULL_PTR);
        (void)GIC_IntSrcDis((uint32_t)GIC_UART0 + ucCh);

        UART_RegWrite(ucCh, UART_REG_ICR, 0x7FF);
    }
}

SALRetCode_t GPIO_MfioCfg(uint32_t uiPeriSel, uint32_t uiPeriType, uint32_t uiChSel,
              uint32_t uiChNum)
{
    uint32_t base_val;
    uint32_t set_val;
    uint32_t clear_bit;
    uint32_t comp_val;

    if (uiPeriSel == GPIO_MFIO_CFG_PERI_SEL0) {
        if (uiChSel == GPIO_MFIO_CFG_CH_SEL0) {
            if (mfio_ch_cfg_flag[0] == 0) {
                /* clear bit */
                base_val = sys_read32(GPIO_MFIO_CFG);
                clear_bit = base_val &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL0) &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL0);
                sys_write32(clear_bit, GPIO_MFIO_CFG);

                base_val = sys_read32(GPIO_MFIO_CFG);
                set_val =
                    base_val |
                    ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL0) |
                    ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL0);
                sys_write32(set_val, GPIO_MFIO_CFG);
                comp_val = sys_read32(GPIO_MFIO_CFG);

                if (comp_val != set_val) {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[0] = 1;
            } else {
                return SAL_RET_FAILED;
            }
        } else {
            return SAL_RET_FAILED;
        }
    } else if (uiPeriSel == GPIO_MFIO_CFG_PERI_SEL1) {
        if (uiChSel == GPIO_MFIO_CFG_CH_SEL1) {
            if (mfio_ch_cfg_flag[1] == 0) {
                /* clear bit */
                base_val = sys_read32(GPIO_MFIO_CFG);
                clear_bit = base_val &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL1) &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL1);
                sys_write32(clear_bit, GPIO_MFIO_CFG);

                base_val = sys_read32(GPIO_MFIO_CFG);
                set_val =
                    base_val |
                    ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL1) |
                    ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL1);
                sys_write32(set_val, GPIO_MFIO_CFG);
                comp_val = sys_read32(GPIO_MFIO_CFG);

                if (comp_val != set_val) {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[1] = 1;
            } else {
                return SAL_RET_FAILED;
            }
        } else {
            return SAL_RET_FAILED;
        }

    } else if (uiPeriSel == GPIO_MFIO_CFG_PERI_SEL2) {
        if (uiChSel == GPIO_MFIO_CFG_CH_SEL2) {
            if (mfio_ch_cfg_flag[2] == 0) {
                /* clear bit */
                base_val = sys_read32(GPIO_MFIO_CFG);
                clear_bit = base_val &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL2) &
                        ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL2);
                sys_write32(clear_bit, GPIO_MFIO_CFG);

                base_val = sys_read32(GPIO_MFIO_CFG);
                set_val =
                    base_val |
                    ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL2) |
                    ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL2);
                sys_write32(set_val, GPIO_MFIO_CFG);
                comp_val = sys_read32(GPIO_MFIO_CFG);

                if (comp_val != set_val) {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[2] = 1;
            } else {
                return SAL_RET_FAILED;
            }
        } else {
            return SAL_RET_FAILED;
        }
    } else {
        return SAL_RET_FAILED;
    }

    return SAL_RET_SUCCESS;
}

static SALRetCode_t UART_ClearGpio(uint8_t ucCh)
{
    uint32_t gpio_Tx = 0;
    uint32_t gpio_Rx = 0;
    uint32_t gpio_Cts = 0;
    uint32_t gpio_Rts = 0;
    SALRetCode_t ret, ret1, ret2;

    ret = SAL_RET_SUCCESS;
    ret1 = SAL_RET_SUCCESS;
    ret2 = SAL_RET_SUCCESS;

    if (ucCh >= UART_CH_MAX) {
        ret = SAL_RET_FAILED;
    } else {
        gpio_Tx = uart[ucCh].sPort.bPortTx;
        gpio_Rx = uart[ucCh].sPort.bPortRx;

        /* Reset gpio */
        ret1 = GPIO_Config(gpio_Tx, GPIO_FUNC(0UL));
        ret2 = GPIO_Config(gpio_Rx, GPIO_FUNC(0UL));

        if ((ret1 != SAL_RET_SUCCESS) || (ret2 != SAL_RET_SUCCESS)) {
            ret = SAL_RET_FAILED;
        } else {
            if (uart[ucCh].sCtsRts == ON) {
                gpio_Cts = uart[ucCh].sPort.bPortCts;
                gpio_Rts = uart[ucCh].sPort.bPortRts;

                ret1 = GPIO_Config(gpio_Cts, GPIO_FUNC(0UL));
                ret2 = GPIO_Config(gpio_Rts, GPIO_FUNC(0UL));

                if ((ret1 != SAL_RET_SUCCESS) || (ret2 != SAL_RET_SUCCESS)) {
                    ret = SAL_RET_FAILED;
                }
            }
        }

        if ((ret != SAL_RET_FAILED) && (ucCh >= UART_CH3)) {
            /* Reset MFIO Configuration */
            if (ucCh == UART_CH3) {
                ret = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL0, GPIO_MFIO_DISABLE,
                           GPIO_MFIO_CFG_CH_SEL0, GPIO_MFIO_CH0);
            } else if (ucCh == UART_CH4) {
                ret = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL1, GPIO_MFIO_DISABLE,
                           GPIO_MFIO_CFG_CH_SEL1, GPIO_MFIO_CH0);
            } else if (ucCh == UART_CH5) {
                ret = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL2, GPIO_MFIO_DISABLE,
                           GPIO_MFIO_CFG_CH_SEL2, GPIO_MFIO_CH0);
            }
        }
    }

    return ret;
}

void GDMA_ChannelDisable(gdma_information_t *sDmacon)
{
    uint32_t uiAddr;
    uint32_t uiValue;
    uint32_t uiMaxCount = 0U;

    if (sDmacon != NULL_PTR) {
        /* checking channel enabled status */
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_ECR; /* Enabled Channel Register */
        uiValue = sys_read32(uiAddr);
        if ((uiValue & ((uint32_t)1UL << sDmacon->iCh)) != 0UL) { /* Currently the channel is enabled */
            uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);
            uiValue = sys_read32(uiAddr);
            uiValue |= (1UL << 18UL); /* ignore subsequent source DMA requests */
            uiValue &= ~(1UL << 0UL); /* channel disabled */
            sys_write32(uiValue, uiAddr);

            /* break loop if the FIFO is not empty after 2ms. time out.
               6250 loops for 1ms can be changed depending on Cortex-R5 Single Core
               clock speed. 6250 loops for 1ms is based on 600MHz which is max clock
               speed for Cortex-R5 clock speed lower than 600MHz is okay because 1ms is
               guaranteed for 6250 loop to have enough time that isr is complete after
               FIFO is empty. if it is higher than 600MHz, isr can be complete too fast
               before FIFO is empty. */

            while (uiMaxCount <= 12500UL) { /* 12500 : 2ms, 6250 : 1ms */
                /* check the Active flag
                   0 = there is no data in the FIFO of the channel */
                uiAddr =
                    GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);

                if ((sys_read32(uiAddr) & (0x20000UL)) == 0UL) {
                    break;
                }
                uiMaxCount += 1UL;
            }
        }
    }
}

static SALRetCode_t UART_Reset(uint8_t ucCh)
{
    SALRetCode_t tRet;
    int32_t iRet;
    int32_t iClkBusId;

    tRet = SAL_RET_SUCCESS;
    iClkBusId = (int32_t)CLOCK_IOBUS_UART0 + (int32_t)ucCh;

    /* SW reset */
    iRet = CLOCK_SetSwReset(iClkBusId, TRUE);

    if (iRet != (int32_t)NULL) {
        tRet = SAL_RET_FAILED;
    } else {
        /* Bit Clear */
        iRet = CLOCK_SetSwReset(iClkBusId, FALSE);

        if (iRet != (int32_t)NULL) {
            tRet = SAL_RET_FAILED;
        }
    }

    return tRet;
}

void uart_close(uint8_t channel)
{
    int32_t iClkBusId;
    SALRetCode_t ret;

    if (channel < UART_CH_MAX) {
        /* Disable the UART Interrupt */
        if (uart[channel].sOpMode == UART_INTR_MODE) {
            UART_DisableInterrupt(channel);
        }

        /* Disable the UART controller Bus clock */
        iClkBusId = (int32_t)CLOCK_IOBUS_UART0 + (int32_t)channel;
        (void)CLOCK_SetIobusPwdn(iClkBusId, TRUE);

        if (uart[channel].sOpMode == UART_DMA_MODE) {
            /* Disable the GDMA controller Bus clock */
            iClkBusId = (int32_t)CLOCK_IOBUS_DMA_CON0 + (int32_t)channel;
            (void)CLOCK_SetIobusPwdn(iClkBusId, TRUE);
        }

        ret = UART_ClearGpio(channel);

        /* Disable the UART ch */
        sys_write32((uint32_t)NULL,
                MCU_BSP_UART_BASE + (0x10000UL * (channel)) + UART_REG_CR);

        /* Disable the GDMA */
        if (uart[channel].sOpMode == UART_DMA_MODE) {
            GDMA_ChannelDisable(&uart[channel].sRxDma);
            GDMA_ChannelDisable(&uart[channel].sTxDma);
        }

        /* Initialize UART Structure */
        memset(&uart[channel], 0, sizeof(uart_status_t));

        /* UART SW Reset */
        (void)UART_Reset(channel);
    }
}

static void UART_StatusInit(uint8_t ucCh)
{
    uart[ucCh].sIsProbed = OFF;
    uart[ucCh].sBase = UART_GET_BASE(ucCh);
    uart[ucCh].sCh = ucCh;
    uart[ucCh].sOpMode = UART_POLLING_MODE;
    uart[ucCh].sCtsRts = 0;
    uart[ucCh].sWordLength = WORD_LEN_5;
    uart[ucCh].s2StopBit = 0;
    uart[ucCh].sParity = PARITY_SPACE;

    /* Interrupt mode init */
    uart[ucCh].sTxIntr.iXmitBuf = NULL;
    uart[ucCh].sTxIntr.iHead = -1;
    uart[ucCh].sTxIntr.iTail = -1;
    uart[ucCh].sTxIntr.iSize = 0;
    uart[ucCh].sRxIntr.iXmitBuf = NULL;
    uart[ucCh].sRxIntr.iHead = -1;
    uart[ucCh].sRxIntr.iTail = -1;
    uart[ucCh].sRxIntr.iSize = 0;

    /* DMA mode init */
    uart[ucCh].sTxDma.iCon = 0xFF;
    uart[ucCh].sTxDma.iCh = 0xFF;
    uart[ucCh].sTxDma.iSrcAddr = NULL;
    uart[ucCh].sTxDma.iDestAddr = NULL;
    uart[ucCh].sTxDma.iBufSize = GDMA_BUFF_SIZE;
    uart[ucCh].sTxDma.iTransSize = 0;
    uart[ucCh].sRxDma.iCon = 0xFF;
    uart[ucCh].sRxDma.iCh = 0xFF;
    uart[ucCh].sRxDma.iSrcAddr = NULL;
    uart[ucCh].sRxDma.iDestAddr = NULL;
    uart[ucCh].sRxDma.iBufSize = GDMA_BUFF_SIZE;
    uart[ucCh].sRxDma.iTransSize = 0;
}

SALRetCode_t GPIO_PerichSel(uint32_t uiPerichSel, uint32_t uiCh)
{
    uint32_t peri_sel_addr;
    uint32_t clear_bit;
    uint32_t set_bit;
    uint32_t base_val;
    uint32_t comp_val;

    peri_sel_addr = GPIO_PERICH_SEL;
    base_val = sys_read32(peri_sel_addr);

    if (uiPerichSel < GPIO_PERICH_SEL_I2SSEL_0) {
        if (uiCh < 2) {
            /* clear bit */
            clear_bit = base_val & ~((0x1UL) << uiPerichSel);
            sys_write32(clear_bit, peri_sel_addr);
            /* set bit */
            base_val = sys_read32(peri_sel_addr);
            set_bit = base_val | ((uiCh & 0x1UL) << uiPerichSel);
            sys_write32(set_bit, peri_sel_addr);
            comp_val = sys_read32(peri_sel_addr);

            if (comp_val != set_bit) {
                return SAL_RET_FAILED;
            }
        } else {
            return SAL_RET_FAILED;
        }
    } else {
        if (uiCh < 4) {
            /* clear bit */
            clear_bit = base_val & ~((0x3UL) << uiPerichSel);
            sys_write32(clear_bit, peri_sel_addr);
            /* set bit */
            base_val = sys_read32(peri_sel_addr);
            set_bit = base_val | ((uiCh & 0x3UL) << uiPerichSel);
            sys_write32(set_bit, peri_sel_addr);
            comp_val = sys_read32(peri_sel_addr);

            if (comp_val != set_bit) {
                return SAL_RET_FAILED;
            }
        } else {
            return SAL_RET_FAILED;
        }
    }

    return SAL_RET_SUCCESS;
}

static int32_t UART_SetGpio(uint8_t ucCh, const uart_board_port_t *psInfo)
{
    int32_t ret;
    SALRetCode_t ret1;
    SALRetCode_t ret2;
    SALRetCode_t ret3;
    SALRetCode_t ret4;
    SALRetCode_t retCfg;

    ret = -2;
    retCfg = SAL_RET_FAILED;

    if (psInfo != NULL_PTR) {
        /* set port controller, channel */
        switch (ucCh) {
        case UART_CH0:
            retCfg = GPIO_PerichSel(GPIO_PERICH_SEL_UARTSEL_0, psInfo->bPortCH);
            break;
        case UART_CH1:
            retCfg = GPIO_PerichSel(GPIO_PERICH_SEL_UARTSEL_1, psInfo->bPortCH);
            break;
        case UART_CH2:
            retCfg = GPIO_PerichSel(GPIO_PERICH_SEL_UARTSEL_2, psInfo->bPortCH);
            break;
        case UART_CH3:
            retCfg = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL0, GPIO_MFIO_UART3,
                          GPIO_MFIO_CFG_CH_SEL0, psInfo->bPortCH);
            break;
        case UART_CH4:
            retCfg = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL1, GPIO_MFIO_UART4,
                          GPIO_MFIO_CFG_CH_SEL1, psInfo->bPortCH);
            break;
        case UART_CH5:
            retCfg = GPIO_MfioCfg(GPIO_MFIO_CFG_PERI_SEL2, GPIO_MFIO_UART5,
                          GPIO_MFIO_CFG_CH_SEL2, psInfo->bPortCH);
            break;
        default:
            retCfg = SAL_RET_FAILED;
            break;
        }

        if (retCfg != SAL_RET_FAILED) {
            /* set debug port */
            ret1 = GPIO_Config(psInfo->bPortTx, (psInfo->bPortFs)); /* TX */
            ret2 = GPIO_Config(psInfo->bPortRx, (psInfo->bPortFs | GPIO_INPUT |
                                 GPIO_INPUTBUF_EN)); /* RX */

            uart[ucCh].sPort.bPortCfg = psInfo->bPortCfg;
            uart[ucCh].sPort.bPortTx = psInfo->bPortTx;
            uart[ucCh].sPort.bPortRx = psInfo->bPortRx;
            uart[ucCh].sPort.bPortFs = psInfo->bPortFs;

            if (uart[ucCh].sCtsRts != 0UL) {
                ret3 = GPIO_Config(psInfo->bPortRts, psInfo->bPortFs); /* RTS */
                ret4 = GPIO_Config(psInfo->bPortCts, psInfo->bPortFs); /* CTS */

                if ((ret1 != SAL_RET_SUCCESS) || (ret2 != SAL_RET_SUCCESS) ||
                    (ret3 != SAL_RET_SUCCESS) || (ret4 != SAL_RET_SUCCESS)) {
                    ret = -1;
                } else {
                    uart[ucCh].sPort.bPortCts = psInfo->bPortCts;
                    uart[ucCh].sPort.bPortRts = psInfo->bPortRts;
                }
            } else if ((ret1 != SAL_RET_SUCCESS) || (ret2 != SAL_RET_SUCCESS)) {
                ret = -1;
            } else {
                ret = 0;
            }
        }
    } else {
        ret = -1;
    }

    return ret;
}

static int32_t UART_SetPortConfig(uint8_t ucCh, uint32_t uiPort)
{
    uint32_t idx;
    int32_t ret = 0;
    static const uart_board_port_t board_serial[UART_PORT_TBL_SIZE] = {
        {0UL, GPIO_GPA(28UL), GPIO_GPA(29UL), TCC_GPNONE, TCC_GPNONE, GPIO_FUNC(1UL),
         GPIO_PERICH_CH0}, /* CTL_0, CH_0 */
        {1UL, GPIO_GPC(16UL), GPIO_GPC(17UL), GPIO_GPC(18UL), GPIO_GPC(19UL),
         GPIO_FUNC(2UL), GPIO_PERICH_CH1}, /* CTL_0, CH_1 */

        {2UL, GPIO_GPB(8UL), GPIO_GPB(9UL), GPIO_GPB(10UL), GPIO_GPB(11UL), GPIO_FUNC(1UL),
         GPIO_PERICH_CH0}, /* CTL_1, CH_0 */
        {3UL, GPIO_GPA(6UL), GPIO_GPA(7UL), GPIO_GPA(8UL), GPIO_GPA(9UL), GPIO_FUNC(2UL),
         GPIO_PERICH_CH1}, /* CTL_1, CH_1 */

        {4UL, GPIO_GPB(25UL), GPIO_GPB(26UL), GPIO_GPB(27UL), GPIO_GPB(28UL),
         GPIO_FUNC(1UL), GPIO_PERICH_CH0}, /* CTL_2, CH_0 */
        {5UL, GPIO_GPC(0UL), GPIO_GPC(1UL), GPIO_GPC(2UL), GPIO_GPC(3UL), GPIO_FUNC(2UL),
         GPIO_PERICH_CH1}, /* CTL_2, CH_1 */

        {6UL, GPIO_GPA(16UL), GPIO_GPA(17UL), GPIO_GPA(18UL), GPIO_GPA(19UL),
         GPIO_FUNC(3UL), GPIO_MFIO_CH0}, /* CTL_3, CH_0 */
        {7UL, GPIO_GPB(0UL), GPIO_GPB(1UL), GPIO_GPB(2UL), GPIO_GPB(3UL), GPIO_FUNC(3UL),
         GPIO_MFIO_CH1}, /* CTL_3, CH_1 */
        {8UL, GPIO_GPC(4UL), GPIO_GPC(5UL), GPIO_GPC(6UL), GPIO_GPC(7UL), GPIO_FUNC(3UL),
         GPIO_MFIO_CH2}, /* CTL_3, CH_2 */
        {9UL, GPIO_GPK(11UL), GPIO_GPK(12UL), GPIO_GPK(13UL), GPIO_GPK(14UL),
         GPIO_FUNC(3UL), GPIO_MFIO_CH3}, /* CTL_3, CH_3 */

        {10UL, GPIO_GPA(20UL), GPIO_GPA(21UL), GPIO_GPA(22UL), GPIO_GPA(23UL),
         GPIO_FUNC(3UL), GPIO_MFIO_CH0}, /* CTL_4, CH_0 */
        {11UL, GPIO_GPB(4UL), GPIO_GPB(5UL), GPIO_GPB(6UL), GPIO_GPB(7UL), GPIO_FUNC(3UL),
         GPIO_MFIO_CH1}, /* CTL_4, CH_1 */
        {12UL, GPIO_GPC(8UL), GPIO_GPC(9UL), GPIO_GPC(10UL), GPIO_GPC(11UL), GPIO_FUNC(3UL),
         GPIO_MFIO_CH2}, /* CTL_4, CH_2 */

        {13UL, GPIO_GPA(24UL), GPIO_GPA(25UL), GPIO_GPA(26UL), GPIO_GPA(27UL),
         GPIO_FUNC(3UL), GPIO_MFIO_CH0}, /* CTL_5, CH_0 */
        {14UL, GPIO_GPB(8UL), GPIO_GPB(9UL), GPIO_GPB(10UL), GPIO_GPB(11UL), GPIO_FUNC(3UL),
         GPIO_MFIO_CH1}, /* CTL_5, CH_1 */
        {15UL, GPIO_GPC(12UL), GPIO_GPC(13UL), GPIO_GPC(14UL), GPIO_GPC(15UL),
         GPIO_FUNC(3UL), GPIO_MFIO_CH2}, /* CTL_5, CH_2 */
    };

    if ((uiPort < UART_PORT_CFG_MAX) && (ucCh < UART_CH_MAX)) {
        for (idx = 0UL; idx < UART_PORT_CFG_MAX; idx++) {
            if (board_serial[idx].bPortCfg == uiPort) {
                ret = UART_SetGpio(ucCh, &board_serial[idx]);
                break;
            }
        }
    }

    return ret;
}

static int32_t UART_SetBaudRate(uint8_t ucCh,
                uint32_t uiBaud) /* (uint32_t => int32_t)return type mismatched */
{
    uint32_t u32_div;
    uint32_t mod;
    uint32_t brd_i;
    uint32_t brd_f;
    uint32_t pclk;
    int32_t ret;

    if (ucCh >= UART_CH_MAX) {
        ret = -1;
    } else {
        /* Read the peri clock */
        pclk = CLOCK_GetPeriRate((int32_t)CLOCK_PERI_UART0 + (int32_t)ucCh);

        if (pclk == 0UL) {
            ret = -1;
        } else {
            /* calculate integer baud rate divisor */
            u32_div = 16UL * uiBaud;
            brd_i = pclk / u32_div;
            UART_RegWrite(ucCh, UART_REG_IBRD, brd_i);

            /* calculate faction baud rate divisor */
            /* NOTICE : fraction maybe need sampling */
            uiBaud &= 0xFFFFFFU;
            mod = (pclk % (16UL * uiBaud)) & 0xFFFFFFU;
            u32_div = ((((1UL << 3UL) * 16UL) * mod) / (16UL * uiBaud));
            brd_f = u32_div / 2UL;
            UART_RegWrite(ucCh, UART_REG_FBRD, brd_f);
            ret = (int32_t)SAL_RET_SUCCESS;
        }
    }
    return ret;
}

static int32_t UART_SetChannelConfig(uart_param_t *pUartCfg)
{
    uint8_t ucCh;
    uint8_t ucWordLength = (uint8_t)pUartCfg->word_length;
    uint32_t cr_data = 0;
    uint32_t lcr_data = 0;
    int32_t ret;
    int32_t iClkBusId;
    int32_t iClkPeriId;

    ucCh = pUartCfg->channel;
    /* Enable the UART controller peri clock */
    iClkBusId = (int32_t)CLOCK_IOBUS_UART0 + (int32_t)ucCh;
    (void)CLOCK_SetIobusPwdn(iClkBusId, SALDisabled);
    iClkPeriId = (int32_t)CLOCK_PERI_UART0 + (int32_t)ucCh;
    ret = CLOCK_SetPeriRate(iClkPeriId, UART_DEBUG_CLK);
    (void)CLOCK_EnablePeri(iClkPeriId);

    if (ret == 0) {
        (void)UART_SetBaudRate(ucCh, pUartCfg->baud_rate);

        /* line control setting */
        /* Word Length */
        ucWordLength &= 0x3U;
        pUartCfg->word_length = (uart_word_len_t)ucWordLength;
        lcr_data |= UART_LCRH_WLEN((uint32_t)pUartCfg->word_length);

        /* Enables FIFOs */
        if (pUartCfg->fifo == ENABLE_FIFO) {
            lcr_data |= UART_LCRH_FEN;
        }

        /* Two Stop Bits */
        if (pUartCfg->stop_bit == ON) {
            lcr_data |= UART_LCRH_STP2;
        }

        /* Parity Enable */
        switch (pUartCfg->parity) {
        case PARITY_SPACE:
            lcr_data &= ~(UART_LCRH_PEN);
            break;
        case PARITY_EVEN:
            lcr_data |= ((UART_LCRH_PEN | UART_LCRH_EPS) & ~(UART_LCRH_SPS));
            break;
        case PARITY_ODD:
            lcr_data |= ((UART_LCRH_PEN & ~(UART_LCRH_EPS)) & ~(UART_LCRH_SPS));
            break;
        case PARITY_MARK:
            lcr_data |= ((UART_LCRH_PEN & ~(UART_LCRH_EPS)) | UART_LCRH_SPS);
            break;
        default:
            break;
        }

        UART_RegWrite(ucCh, UART_REG_LCRH, lcr_data);

        /* control register setting */
        cr_data = UART_CR_EN;
        cr_data |= UART_CR_TXE;
        cr_data |= UART_CR_RXE;

        if (uart[ucCh].sCtsRts != 0UL) { /* brace */
            cr_data |= (UART_CR_RTSEN | UART_CR_CTSEN);
        }

        UART_RegWrite(ucCh, UART_REG_CR, cr_data);
    }

    return ret;
}

static void UART_InterruptTxProbe(uint8_t ucCh)
{
    if (ucCh < UART_CH_MAX) {
        uart[ucCh].sTxIntr.iXmitBuf = uart_buff[ucCh][UART_MODE_TX];
        uart[ucCh].sTxIntr.iSize = (int32_t)UART_BUFF_SIZE;
        uart[ucCh].sTxIntr.iHead = 0;
        uart[ucCh].sTxIntr.iTail = 0;
    }
}

static void UART_InterruptRxProbe(uint8_t ucCh)
{
    if (ucCh < UART_CH_MAX) {
        uart[ucCh].sRxIntr.iXmitBuf = uart_buff[ucCh][UART_MODE_RX];
        uart[ucCh].sRxIntr.iSize = (int32_t)UART_BUFF_SIZE;
        uart[ucCh].sRxIntr.iHead = 0;
        uart[ucCh].sRxIntr.iTail = 0;
    }
}

static void UART_InterruptProbe(uint8_t ucCh)
{
    UART_InterruptTxProbe(ucCh);
    UART_InterruptRxProbe(ucCh);
}

SALRetCode_t GIC_IntSrcEn(uint32_t uiIntId)
{
    SALRetCode_t ucRet;
    uint32_t uiRegOffset;
    uint32_t uiBit;

    ucRet = (SALRetCode_t)SAL_RET_FAILED;
    uiRegOffset = 0;
    uiBit = 0;

    if (uiIntId < GIC_INT_SRC_CNT) {
        uiRegOffset = (uiIntId >> 5u); /* Calculate the register offset. */
        uiBit = (uiIntId & 0x1Fu);     /* Mask bit ID.     */

        GIC_DIST->dISENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);

        if (GICIsrTable[uiIntId].ifpIsBothEdge == (1UL)) {
            uiRegOffset =
                ((uiIntId + 10UL) >> 5UL);   /* Calculate the register offset.   */
            uiBit = ((uiIntId + 10UL) & 0x1FUL); /* Mask bit ID. */

            GIC_DIST->dISENABLERn[uiRegOffset] = ((uint32_t)1UL << uiBit);
        }

        FR_CoreMB();
        ucRet = (SALRetCode_t)SAL_RET_SUCCESS;
    }

    return ucRet;
}

static uint32_t UART_RegRead(uint8_t ucCh, uint32_t uiAddr)
{
    uint32_t uiRet;
    uint32_t uiBaseAddr;
    uint32_t uiRegAddr;

    uiRet = 0;

    if (uart[ucCh].sBase == 0UL) {
        uart[ucCh].sBase = UART_GET_BASE(ucCh);
    }

    uiBaseAddr = uart[ucCh].sBase & 0xAFFFFFFFU;
    uiAddr &= 0xFFFFU;
    uiRegAddr = uiBaseAddr + uiAddr;
    uiRet = sys_read32(uiRegAddr);

    return uiRet;
}

static void UART_EnableInterrupt(uint8_t ucCh, uint32_t uiPriority, uint8_t ucFIFO,
                 GICIsrFunc fnCallback)
{
    uint32_t i;
    uint32_t im = 0UL;

    if (ucCh < UART_CH_MAX) {
        (void)GIC_IntVectSet((uint32_t)GIC_UART0 + ucCh, uiPriority,
                     GIC_INT_TYPE_LEVEL_HIGH, fnCallback, &(uart[ucCh]));
        (void)GIC_IntSrcEn((uint32_t)GIC_UART0 + ucCh);

        UART_RegWrite(ucCh, UART_REG_ICR, UART_INT_RXIS | UART_INT_TXIS | UART_INT_RTIS);

        for (i = 0UL; i < (UART_RX_FIFO_SIZE * 2UL); ++i) {
            if ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_RXFF) != 0UL) {
                break;
            }

            (void)UART_RegRead(ucCh, UART_REG_DR);
        }

        if (ucFIFO == ENABLE_FIFO) {
            im = UART_INT_RTIS;
        }

        im |= UART_INT_RXIS;

        UART_RegWrite(ucCh, UART_REG_IMSC, im);
    }
}

uint32_t MPU_GetDMABaseAddress(void)
{
    /* 1. Ths "__nc_dmastart" value already contains the physical base(0xC0000000). Please
     * reference the linker(your) script file(.ld)
     * 2. The dma address is valid only physical address(memory and peripheral point of view).
     */

    uint32_t uiDMAStart;

    uiDMAStart = (uint32_t)(&__nc_dmastart); /* Physical Offset for 512KB SRAM for execution */
                         /* zero base */

    return uiDMAStart;
}

void GDMA_SetFlowControl(gdma_information_t *sDmacon, uint32_t uiFlow)
{
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        uiValue &= ~(0x7UL << 11UL);
        uiValue |= ((uiFlow & 0x7UL) << 11UL);
        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_SetAddrIncrement(gdma_information_t *sDmacon, uint8_t ucDestInc, uint32_t uiSrcInc)
{
    uint32_t inc;
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CON(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        /* clear desc, src increment */
        uiValue &= ~(0x3UL << 26UL);

        inc = (((uint32_t)ucDestInc & 0x1UL) << 1UL) | (uiSrcInc & 0x1UL);
        uiValue |= (inc << 26UL);

        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_SetBurstSize(gdma_information_t *sDmacon, uint8_t ucDestBurst, uint32_t uiSrcBurst)
{
    uint32_t burst_size;
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CON(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        /* clear desc, src burst size */
        uiValue &= ~(0x3FUL << 12UL);
        burst_size = (((uint32_t)ucDestBurst & 0x7UL) << 3UL) | (uiSrcBurst & 0x7UL);
        uiValue |= (burst_size << 12UL);

        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_SetPeri(gdma_information_t *sDmacon, uint8_t ucDestPeri, uint8_t ucSrcPeri)
{
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        uiValue &= ~(0x7FEUL);
        uiValue |= ((uint32_t)ucDestPeri & 0xFUL) << 6UL;
        uiValue |= ((uint32_t)ucSrcPeri & 0xFUL) << 1UL;
        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_SetSrcAddr(gdma_information_t *sDmacon, uint32_t uiAddr)
{
    uint32_t uiRegAddr;

    if (sDmacon != NULL_PTR) {
        uiRegAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_SRC_ADDR(sDmacon->iCh);
        sys_write32(uiAddr, uiRegAddr);
    }
}

void GDMA_SetDestAddr(gdma_information_t *sDmacon, uint32_t uiAddr)
{
    uint32_t uiRegAddr;

    if (sDmacon != NULL_PTR) {
        uiRegAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_DST_ADDR(sDmacon->iCh);
        sys_write32(uiAddr, uiRegAddr);
    }
}

void GDMA_SetTransferWidth(gdma_information_t *sDmacon, uint8_t ucDestWidth, uint32_t uiSrcWidth)
{
    uint32_t transfer_width;
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CON(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        /* clear desc, src transfer width */
        uiValue &= ~(0x3FUL << 18UL);
        transfer_width = (((uint32_t)ucDestWidth & 0x7UL) << 3UL) | (uiSrcWidth & 0x7UL);
        uiValue |= (transfer_width << 18UL);

        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_SetTransferSize(gdma_information_t *sDmacon, uint32_t uiTransferSize)
{
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CON(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        /* clear transfer size */
        uiValue &= ~(0xFFFUL);
        uiValue |= (uiTransferSize & 0xFFFUL);
        sys_write32(uiValue, uiAddr);
    }
}

void GDMA_InterruptEnable(gdma_information_t *sDmacon)
{
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        /* Enable terminal count interrupt */
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CON(sDmacon->iCh);
        uiValue = (uint32_t)sys_read32(uiAddr) | ((uint32_t)1UL << 31UL);

        sys_write32(uiValue, uiAddr);

        /* Mask terminal count interrupt */
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);
        uiValue = (uint32_t)sys_read32(uiAddr) | ((uint32_t)1UL << 15UL);
        sys_write32(uiValue, uiAddr);
    }
}

static int32_t UART_DmaTxEnable(uint8_t ucCh, uint32_t uiSize, const gdma_information_t *psDmacon)
{
    int32_t ret;

    if (ucCh >= UART_CH_MAX) {
        ret = -1;
    } else {
        uart[ucCh].sTxDma.iSrcAddr = (uint8_t *)(psDmacon->iSrcAddr);
        uart[ucCh].sTxDma.iDestAddr = (uint8_t *)(psDmacon->iDestAddr);
        GDMA_SetSrcAddr(&uart[ucCh].sTxDma, (uint32_t)(uart[ucCh].sTxDma.iSrcAddr));
        GDMA_SetDestAddr(&uart[ucCh].sTxDma, (uint32_t)(uart[ucCh].sTxDma.iDestAddr));
        GDMA_InterruptEnable(&uart[ucCh].sTxDma);
        GDMA_SetTransferWidth(&uart[ucCh].sTxDma, GDMA_TRANSFER_SIZE_BYTE,
                      GDMA_TRANSFER_SIZE_BYTE);
        GDMA_SetTransferSize(&uart[ucCh].sTxDma, uiSize);
        ret = (int32_t)SAL_RET_SUCCESS;
    }

    return ret;
}

static void GDMA_ISR(void *pArg)
{
    uint32_t uiIntStatus;
    uint32_t uiTCIntStatus;
    uint32_t uiErrIntStatus;
    uint32_t uiAddr;
    GDMAController_t *pConTable;
    gdma_information_t *dmacon;
    uint32_t uiIndex;

    pConTable = (GDMAController_t *)pArg;
    dmacon = NULL;

    if (pArg != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(pConTable->cController) + GDMA_INTSR;
        uiIntStatus = sys_read32(uiAddr);
        uiAddr = GDMA_CON_BASE(pConTable->cController) + GDMA_ITCSR;
        uiTCIntStatus = sys_read32(uiAddr);
        uiAddr = GDMA_CON_BASE(pConTable->cController) + GDMA_IESR;
        uiErrIntStatus = sys_read32(uiAddr);
        for (uiIndex = 0; uiIndex < GDMA_CH_MAX; uiIndex++) {
            /* check interrupt status */
            if ((uiIntStatus & ((uint32_t)1UL << uiIndex)) != 0UL) {
                /* check terminal count interrupt status. */
                if ((uiTCIntStatus & ((uint32_t)1UL << uiIndex)) != 0UL) {
                    dmacon = (gdma_information_t *)pConTable->cCh[uiIndex];
                    if (dmacon != NULL_PTR) { /* Codesonar : Null Test After Dereference */
                        /* Clear terminal count interrupt */
                        uiAddr = GDMA_CON_BASE(dmacon->iCon) + GDMA_ITCCR;
                        sys_write32(((uint32_t)1UL << uiIndex), uiAddr);
                        if (dmacon->fpIsrCallbackForComplete != NULL_PTR) {
                            dmacon->fpIsrCallbackForComplete( (void *)dmacon);
                        }
                    }
                }
                /* check error interrupt status. */
                if ((uiErrIntStatus & ((uint32_t)1UL << uiIndex)) != 0UL) {
                    dmacon = (gdma_information_t *)pConTable->cCh[uiIndex];
                    if (dmacon != NULL_PTR) { /* Codesonar : Null Test After Dereference */
                        /* Clear error interrupt */
                        uiAddr = GDMA_CON_BASE(dmacon->iCon) + GDMA_IECR;
                        sys_write32(((uint32_t)1UL << uiIndex), uiAddr);
                    }
                }
            }
        }
    }
}

int32_t GDMA_Init(gdma_information_t *sDmacon, uint32_t uiIntPrio)
{
    int32_t ret;
    uint32_t uiAddr;
    uint32_t uiValue;
    int32_t iClkId;
    static GDMAController_t gdma_con_table[GDMA_CON_MAX] = {
        {0UL, {NULL, NULL}}, {1UL, {NULL, NULL}}, {2UL, {NULL, NULL}}, {3UL, {NULL, NULL}},
        {4UL, {NULL, NULL}}, {5UL, {NULL, NULL}}, {6UL, {NULL, NULL}}, {7UL, {NULL, NULL}}};

    ret = -1;
    iClkId = 0;

    if (sDmacon != NULL_PTR) {
        iClkId = (int32_t)CLOCK_IOBUS_DMA_CON0 + (int32_t)sDmacon->iCon;
        (void)CLOCK_EnableIobus(iClkId, SALEnabled);

        gdma_con_table[sDmacon->iCon].cController = sDmacon->iCon;
        gdma_con_table[sDmacon->iCon].cCh[sDmacon->iCh] = sDmacon;
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CR;

        if ((sys_read32(uiAddr) & 0x1UL) == 0UL) {
            /* Disable DMA Contorller */
            uiValue = sys_read32(uiAddr);
            sys_write32(uiValue & ~(0x1UL), uiAddr);

            /* Clear Interrupt */
            uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_ITCCR;
            sys_write32(0xFFUL, uiAddr);
            uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_IECR;
            sys_write32(0xFFUL, uiAddr);

            (void)GIC_IntVectSet((uint32_t)GIC_DMA0 + sDmacon->iCon, uiIntPrio,
                         GIC_INT_TYPE_LEVEL_HIGH, (GICIsrFunc)&GDMA_ISR,
                         (void *)&gdma_con_table[sDmacon->iCon]);
            (void)GIC_IntSrcEn((uint32_t)GIC_DMA0 + sDmacon->iCon);

            /* Enable DMA Controller */
            uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CR;
            sys_write32(0x1UL, uiAddr);
            /* ret = (int32_t)SAL_RET_SUCCESS; */
            /* uiGDMAEnabled = 1UL;  */ /* 2 ch per 8 controller */
        }
        ret = (int32_t)SAL_RET_SUCCESS;
    }

    return ret;
}

static void UART_DmaTxProbe(uint8_t ucCh, uint32_t *puiAddr)
{
    uint32_t regData;

    if (ucCh < UART_CH_MAX) {
        uart[ucCh].sTxDma.iCon = ucCh;
        uart[ucCh].sTxDma.iCh = (uint32_t)GDMA_PERI_TX;
        uart[ucCh].sTxDma.iSrcAddr = (uint8_t *)(puiAddr);
        uart[ucCh].sTxDma.iDestAddr = (uint8_t *)(uart[ucCh].sBase);
        uart[ucCh].sTxDma.iBufSize = GDMA_BUFF_SIZE;
        uart[ucCh].sTxDma.iTransSize = 0;

        (void)GDMA_Init(&uart[ucCh].sTxDma, GIC_PRIORITY_NO_MEAN);

        /* Enable Transmit DMA */
        regData = (UART_RegRead(ucCh, UART_REG_DMACR) | UART_DMACR_DMAONERR |
               UART_DMACR_TXDMAE);
        UART_RegWrite(ucCh, UART_REG_DMACR, regData);

        GDMA_SetFlowControl(&uart[ucCh].sTxDma, GDMA_FLOW_TYPE_M2P);
        GDMA_SetAddrIncrement(&uart[ucCh].sTxDma, GDMA_NO_INC, GDMA_INC);
        GDMA_SetBurstSize(&uart[ucCh].sTxDma, GDMA_BURST_SIZE_1, GDMA_BURST_SIZE_1);

        GDMA_SetPeri(&uart[ucCh].sTxDma, (uint8_t)GDMA_PERI_REQ_PORT_UART1_TX, 0U);

        (void)UART_DmaTxEnable(ucCh, uart[ucCh].sTxDma.iBufSize,
                       (const gdma_information_t *)&uart[ucCh].sTxDma);
    }
}

void GDMA_ChannelEnable(gdma_information_t *sDmacon)
{
    uint32_t uiAddr;
    uint32_t uiValue;

    if (sDmacon != NULL_PTR) {
        uiAddr = GDMA_CON_BASE(sDmacon->iCon) + GDMA_CH_CONFIG(sDmacon->iCh);
        uiValue = sys_read32(uiAddr);
        uiValue &= ~(1UL << 18UL); /* enable DMA requests */
        uiValue |= (1UL << 0UL);   /* channel enabled */

        sys_write32(uiValue, uiAddr);
    }
}

static void UART_DmaRxISR(void *pArg)
{
    gdma_information_t *dmacon = NULL_PTR;

    (void)memcpy(&dmacon, &pArg, sizeof(gdma_information_t *));

    if (dmacon != NULL_PTR) {
        GDMA_SetDestAddr(dmacon, (uint32_t)dmacon->iDestAddr);
        GDMA_InterruptEnable(dmacon);
        GDMA_SetTransferSize(dmacon, dmacon->iBufSize);

        GDMA_ChannelEnable(dmacon);
    }
}

static int32_t UART_DmaRxEnable(uint8_t ucCh, uint32_t uiSize, const gdma_information_t *psDmacon)
{
    int32_t ret;

    if (ucCh >= UART_CH_MAX) {
        ret = -1;
    } else {
        uart[ucCh].sRxDma.iSrcAddr = (uint8_t *)(psDmacon->iSrcAddr);
        uart[ucCh].sRxDma.iDestAddr = (uint8_t *)(psDmacon->iDestAddr);

        GDMA_SetSrcAddr(&uart[ucCh].sRxDma, (uint32_t)(uart[ucCh].sRxDma.iSrcAddr));
        GDMA_SetDestAddr(&uart[ucCh].sRxDma, (uint32_t)(uart[ucCh].sRxDma.iDestAddr));
        GDMA_InterruptEnable(&uart[ucCh].sRxDma);
        GDMA_SetTransferWidth(&uart[ucCh].sRxDma, GDMA_TRANSFER_SIZE_BYTE,
                      GDMA_TRANSFER_SIZE_BYTE);
        GDMA_SetTransferSize(&uart[ucCh].sRxDma, uiSize);

        /* Run DMA */
        GDMA_ChannelEnable(&uart[ucCh].sRxDma);
        ret = (int32_t)SAL_RET_SUCCESS;
    }

    return ret;
}

static void UART_DmaRxProbe(uint8_t ucCh, uint32_t *puiAddr)
{
    uint32_t regData;

    if (ucCh < UART_CH_MAX) {
        uart[ucCh].sRxDma.iCon = ucCh;
        uart[ucCh].sRxDma.iCh = (uint32_t)GDMA_PERI_RX;
        uart[ucCh].sRxDma.iSrcAddr = (uint8_t *)(uart[ucCh].sBase);
        uart[ucCh].sRxDma.iDestAddr = (uint8_t *)(puiAddr);
        uart[ucCh].sRxDma.iBufSize = GDMA_BUFF_SIZE;
        uart[ucCh].sRxDma.iTransSize = 0UL;
        uart[ucCh].sRxDma.fpIsrCallbackForComplete = UART_DmaRxISR;

        (void)GDMA_Init(&uart[ucCh].sRxDma, GIC_PRIORITY_NO_MEAN);

        /* Enable Receive DMA */
        regData = (UART_RegRead(ucCh, UART_REG_DMACR) | UART_DMACR_DMAONERR |
               UART_DMACR_RXDMAE);
        UART_RegWrite(ucCh, UART_REG_DMACR, regData);

        GDMA_SetFlowControl(&uart[ucCh].sRxDma, GDMA_FLOW_TYPE_P2M);
        GDMA_SetAddrIncrement(&uart[ucCh].sRxDma, GDMA_INC, GDMA_NO_INC);
        GDMA_SetBurstSize(&uart[ucCh].sRxDma, GDMA_BURST_SIZE_1, GDMA_BURST_SIZE_1);

        GDMA_SetPeri(&uart[ucCh].sRxDma, 0U, (uint8_t)GDMA_PERI_REQ_PORT_UART1_RX);

        (void)UART_DmaRxEnable(ucCh, uart[ucCh].sRxDma.iBufSize,
                       (const gdma_information_t *)&uart[ucCh].sRxDma);
    }
}

static void UART_DmaProbe(uint8_t ucCh)
{
    uint32_t uiDmaRxAddr;
    uint32_t uiDmaTxAddr;
    static uint32_t *dma_tx_buf;
    static uint32_t *dma_rx_buf;

    if (ucCh < UART_CH_MAX) {
        uiDmaRxAddr = MPU_GetDMABaseAddress() + GDMA_ADDRESS_UNIT_CH_RX(ucCh);
        uiDmaTxAddr = MPU_GetDMABaseAddress() + GDMA_ADDRESS_UNIT_CH_TX(ucCh);

        (void)memcpy(&dma_rx_buf, &uiDmaRxAddr, sizeof(uint32_t *));
        (void)memcpy(&dma_tx_buf, &uiDmaTxAddr, sizeof(uint32_t *));

        if ((dma_tx_buf != NULL_PTR) && (dma_rx_buf != NULL_PTR)) {
            (void)memset((void *)dma_rx_buf, 0, GDMA_BUFF_SIZE);
            (void)memset((void *)dma_tx_buf, 0, GDMA_BUFF_SIZE);

            UART_DmaTxProbe(ucCh, dma_tx_buf);
            UART_DmaRxProbe(ucCh, dma_rx_buf);
        }
    }
}

static int32_t UART_Probe(uart_param_t *pUartCfg)
{
    uint8_t ucCh;
    int32_t ret = -1;

    ucCh = pUartCfg->channel;

    if ((ucCh < UART_CH_MAX) && (uart[ucCh].sIsProbed == OFF)) {
        uart[ucCh].sOpMode = pUartCfg->mode;
        uart[ucCh].sCtsRts = pUartCfg->cts_rts;

        /* Set port config */
        ret = UART_SetPortConfig(ucCh, pUartCfg->port_cfg);

        if (ret != -1) {
            ret = UART_SetChannelConfig(pUartCfg);

            if (ret != -1) {
                if (uart[ucCh].sOpMode == UART_INTR_MODE) { /* Configure for interrupt mode */
                    UART_InterruptProbe(ucCh);
                    UART_EnableInterrupt(ucCh, pUartCfg->priority,
                                 pUartCfg->fifo, pUartCfg->callback_fn);
                } else if (uart[ucCh].sOpMode == UART_DMA_MODE) { /* Configure for DMA mode */
                    UART_DmaProbe(ucCh);
                    UART_EnableInterrupt(ucCh, pUartCfg->priority,
                                 pUartCfg->fifo, pUartCfg->callback_fn);
                }

                uart[ucCh].sIsProbed = ON;
            }
        }
    }

    return ret;
}

int32_t UART_Open(uart_param_t *pUartCfg)
{
    uint8_t ucCh = pUartCfg->channel;
    int32_t ret = -1;

    UART_StatusInit(ucCh);

    if (pUartCfg->port_cfg <= UART_PORT_CFG_MAX) {
        ret = UART_Probe(pUartCfg);
    }

    return ret;
}

static int uart_tccvcp_init(const struct device *dev)
{
    const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
    uart_param_t uart_pars;

    uart_pars.channel = dev_cfg->channel;
    uart_pars.priority = GIC_PRIORITY_NO_MEAN;
    uart_pars.baud_rate = dev_cfg->baud_rate;
    uart_pars.mode = UART_POLLING_MODE;
    uart_pars.cts_rts = UART_CTSRTS_OFF;
    uart_pars.port_cfg = (uint8_t)(4U + dev_cfg->channel);
    uart_pars.fifo = DISABLE_FIFO, uart_pars.stop_bit = TWO_STOP_BIT_OFF;
    uart_pars.word_length = WORD_LEN_8;
    uart_pars.parity = PARITY_SPACE;
    uart_pars.callback_fn = NULL_PTR;

    (void)uart_close(uart_pars.channel);
    (void)UART_Open(&uart_pars);

    return 0;
}

static int uart_tccvcp_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
    uint8_t ucCh = dev_cfg->channel;
    uint32_t status;
    uint32_t data;

    if (ucCh >= UART_CH_MAX) {
        return -1;
    } else {
        status = UART_RegRead(ucCh, UART_REG_FR);

        if ((status & UART_FR_RXFE) == 0UL) {
            data = UART_RegRead(ucCh, UART_REG_DR);
            *c = (unsigned char)(data & 0xFFUL);
        }
    }

    return 0;
}

static void uart_tccvcp_poll_out(const struct device *dev, unsigned char c)
{
    const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
    uint8_t ucCh = dev_cfg->channel;

    if (ucCh >= UART_CH_MAX) {
        return;
    } else {
        if ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_TXFF) == 0UL) {
            UART_RegWrite(ucCh, UART_REG_DR, c);
        }
    }
}

static inline bool uart_tccvcp_cfg2ll_parity(uint32_t *mode_reg, enum uart_config_parity parity)
{
    switch (parity) {
    default:
    case PARITY_EVEN:
        *mode_reg |= ((UART_LCRH_PEN | UART_LCRH_EPS) & ~(UART_LCRH_SPS));
        break;
    case PARITY_ODD:
        *mode_reg |= ((UART_LCRH_PEN & ~(UART_LCRH_EPS)) & ~(UART_LCRH_SPS));
        break;
    case PARITY_SPACE:
        *mode_reg &= ~(UART_LCRH_PEN);
        break;
    case PARITY_MARK:
        *mode_reg |= ((UART_LCRH_PEN & ~(UART_LCRH_EPS)) | UART_LCRH_SPS);
        break;
    }

    return true;
}

static inline bool uart_tccvcp_cfg2ll_stopbits(uint32_t *mode_reg,
                           enum uart_config_stop_bits stopbits)
{
    switch (stopbits) {
    default:
    case UART_CFG_STOP_BITS_2:
        *mode_reg |= UART_LCRH_STP2;
        break;
    }

    return true;
}

static inline bool uart_tccvcp_cfg2ll_databits(uint32_t *mode_reg,
                           enum uart_config_data_bits databits)
{
    switch (databits) {
    case UART_CFG_DATA_BITS_9:
        /* Controller doesn't support 5 or 9 data bits */
        return false;
    default:
    case UART_CFG_DATA_BITS_8:
        *mode_reg |= UART_LCRH_WLEN(WORD_LEN_8);
        break;
    case UART_CFG_DATA_BITS_7:
        *mode_reg |= UART_LCRH_WLEN(WORD_LEN_7);
        break;
    case UART_CFG_DATA_BITS_6:
        *mode_reg |= UART_LCRH_WLEN(WORD_LEN_6);
        break;
    case UART_CFG_DATA_BITS_5:
        *mode_reg |= UART_LCRH_WLEN(WORD_LEN_5);
        break;
    }

    return true;
}

static inline bool uart_tccvcp_cfg2ll_hwctrl(uint32_t *modemcr_reg,
                         enum uart_config_flow_control hwctrl)
{
    /*
     * Translate the new flow control configuration to the modem
     * control register's bit [5] (FCM):
     *  0b : no flow control
     *  1b : RTS/CTS
     */

    if (hwctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
        *modemcr_reg |= (UART_CR_RTSEN | UART_CR_CTSEN);
    } else {
        /* Only no flow control or RTS/CTS is supported. */
        return false;
    }

    return true;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_tccvcp_configure(const struct device *dev, const struct uart_config *cfg)
{
    const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
    uint8_t ucCh = dev_cfg->channel;
    uart_param_t uart_cfg;
    int32_t ret;

    uart_cfg.channel = dev_cfg->channel;
    uart_cfg.priority = GIC_PRIORITY_NO_MEAN;
    uart_cfg.baud_rate = cfg->baudrate;
    uart_cfg.mode = UART_POLLING_MODE;
    switch (cfg->flow_ctrl) {
    case UART_CFG_FLOW_CTRL_NONE:
        uart_cfg.cts_rts = UART_CTSRTS_OFF;
        break;
    case UART_CFG_FLOW_CTRL_RTS_CTS:
        uart_cfg.cts_rts = UART_CTSRTS_ON;
        break;
    default:
        uart_cfg.cts_rts = UART_CTSRTS_OFF;
        break;
    }

    uart_cfg.port_cfg = (uint8_t)(4U + dev_cfg->channel);

    switch (cfg->data_bits) {
    case UART_CFG_DATA_BITS_8:
        uart_cfg.word_length = WORD_LEN_8;
        break;
    case UART_CFG_DATA_BITS_7:
        uart_cfg.word_length = WORD_LEN_7;
        break;
    case UART_CFG_DATA_BITS_6:
        uart_cfg.word_length = WORD_LEN_6;
        break;
    case UART_CFG_DATA_BITS_5:
        uart_cfg.word_length = WORD_LEN_5;
        break;
    default:
        uart_cfg.word_length = WORD_LEN_8 + 1;
        break;
    }

    uart_cfg.fifo = DISABLE_FIFO;

    switch (cfg->stop_bits) {
    case UART_CFG_STOP_BITS_2:
        uart_cfg.stop_bit = TWO_STOP_BIT_ON;
        break;
    default:
        uart_cfg.stop_bit = TWO_STOP_BIT_OFF;
        break;
    }

    switch (cfg->parity) {
    case UART_CFG_PARITY_EVEN:
        uart_cfg.parity = PARITY_EVEN;
        break;
    case UART_CFG_PARITY_ODD:
        uart_cfg.parity = PARITY_ODD;
        break;
    case UART_CFG_PARITY_SPACE:
        uart_cfg.parity = PARITY_SPACE;
        break;
    case UART_CFG_PARITY_MARK:
        uart_cfg.parity = PARITY_MARK;
        break;
    default:
        uart_cfg.parity = PARITY_MARK + 1;
        break;
    }

    uart_cfg.callback_fn = NULL_PTR;

    ret = UART_SetChannelConfig(&uart_cfg);
    if (ret == 0) {
        uart[ucCh].sCtsRts = uart_cfg.cts_rts;
        uart[ucCh].s2StopBit = uart_cfg.stop_bit;
        uart[ucCh].sParity = uart_cfg.parity;
        uart[ucCh].sWordLength = uart_cfg.word_length;
        uart[ucCh].baudrate = uart_cfg.baud_rate;
    }

    return ret;
};
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static inline enum uart_config_parity uart_tccvcp_ll2cfg_parity(uint32_t mode_reg)
{
    switch ((mode_reg & 0x7)) {
    case PARITY_EVEN:
    default:
        return UART_CFG_PARITY_EVEN;
    case PARITY_ODD:
        return UART_CFG_PARITY_ODD;
    case PARITY_SPACE:
        return UART_CFG_PARITY_SPACE;
    case PARITY_MARK:
        return UART_CFG_PARITY_MARK;
    case (PARITY_MARK + 1):
        return UART_CFG_PARITY_NONE;
    }
}

static inline enum uart_config_stop_bits uart_tccvcp_ll2cfg_stopbits(uint32_t mode_reg)
{
    switch ((mode_reg)) {
    case UART_CFG_STOP_BITS_2:
        return UART_CFG_STOP_BITS_2;
    default:
        return UART_CFG_STOP_BITS_1;
    }
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_tccvcp_config_get(const struct device *dev, struct uart_config *cfg)
{
    const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
    uint8_t ucCh = dev_cfg->channel;

    /* cfg->baudrate = dev_cfg->baud_rate; */
    cfg->baudrate = uart[ucCh].baudrate;

    switch (uart[ucCh].sCtsRts) {
    case UART_CTSRTS_ON:
        cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
        break;
    case UART_CTSRTS_OFF:
    default:
        cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
        break;
    }

    switch (uart[ucCh].sWordLength) {
    case WORD_LEN_8:
        cfg->data_bits = UART_CFG_DATA_BITS_8;
        break;
    case WORD_LEN_7:
        cfg->data_bits = UART_CFG_DATA_BITS_7;
        break;
    case WORD_LEN_6:
        cfg->data_bits = UART_CFG_DATA_BITS_6;
        break;
    case WORD_LEN_5:
        cfg->data_bits = UART_CFG_DATA_BITS_5;
        break;
    default:
        cfg->data_bits = UART_CFG_DATA_BITS_9;
        break;
    }

    switch (uart[ucCh].s2StopBit) {
    case TWO_STOP_BIT_ON:
        cfg->stop_bits = UART_CFG_STOP_BITS_2;
        break;
    default:
        cfg->stop_bits = UART_CFG_STOP_BITS_0_5;
        break;
    }

    switch (uart[ucCh].sParity) {
    case PARITY_EVEN:
        cfg->parity = UART_CFG_PARITY_EVEN;
        break;
    case PARITY_ODD:
        cfg->parity = UART_CFG_PARITY_ODD;
        break;
    case PARITY_SPACE:
        cfg->parity = UART_CFG_PARITY_SPACE;
        break;
    case PARITY_MARK:
        cfg->parity = UART_CFG_PARITY_MARK;
        break;
    default:
        cfg->parity = UART_CFG_PARITY_NONE;
        break;
    }

    return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static const struct uart_driver_api uart_tccvcp_driver_api = {
    .poll_in = uart_tccvcp_poll_in,
    .poll_out = uart_tccvcp_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_tccvcp_configure,
    .config_get = uart_tccvcp_config_get,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define UART_TCC_VCP_IRQ_CONF_FUNC_SET(port) .irq_config_func = uart_tccvcp_irq_config_##port,

#define UART_TCC_VCP_IRQ_CONF_FUNC(port)                                                           \
    static void uart_tccvcp_irq_config_##port(const struct device *dev)                        \
    {                                                                                          \
        IRQ_CONNECT(DT_INST_IRQN(port), DT_INST_IRQ(port, priority), uart_tccvcp_isr,      \
                DEVICE_DT_INST_GET(port), 0);                                          \
        irq_enable(DT_INST_IRQN(port));                                                    \
    }

#else

#define UART_TCC_VCP_IRQ_CONF_FUNC_SET(port)
#define UART_TCC_VCP_IRQ_CONF_FUNC(port)

#endif /*CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_TCC_VCP_DEV_DATA(port) static struct uart_tccvcp_dev_data_t uart_tccvcp_dev_data_##port

#if CONFIG_PINCTRL
#define UART_TCC_VCP_PINCTRL_DEFINE(port) PINCTRL_DT_INST_DEFINE(port);
#define UART_TCC_VCP_PINCTRL_INIT(port)   .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(port),
#else
#define UART_TCC_VCP_PINCTRL_DEFINE(port)
#define UART_TCC_VCP_PINCTRL_INIT(port)
#endif /* CONFIG_PINCTRL */

#define UART_TCC_VCP_DEV_CFG(port)                                                                 \
    static struct uart_tccvcp_dev_config uart_tccvcp_dev_cfg_##port = {                        \
        DEVICE_MMIO_ROM_INIT(DT_DRV_INST(port)),                                           \
        .sys_clk_freq = DT_INST_PROP(port, clock_frequency),                               \
        .baud_rate = DT_INST_PROP(port, current_speed), .channel = port,                   \
        UART_TCC_VCP_IRQ_CONF_FUNC_SET(port) UART_TCC_VCP_PINCTRL_INIT(port)}

#define UART_TCC_VCP_INIT(port)                                                                    \
    DEVICE_DT_INST_DEFINE(port, uart_tccvcp_init, NULL, &uart_tccvcp_dev_data_##port,          \
                  &uart_tccvcp_dev_cfg_##port, PRE_KERNEL_1,                           \
                  CONFIG_SERIAL_INIT_PRIORITY, &uart_tccvcp_driver_api)

#define UART_TCC_INSTANTIATE(inst)                                                                 \
    UART_TCC_VCP_PINCTRL_DEFINE(inst)                                                          \
    UART_TCC_VCP_IRQ_CONF_FUNC(inst);                                                          \
    UART_TCC_VCP_DEV_DATA(inst);                                                               \
    UART_TCC_VCP_DEV_CFG(inst);                                                                \
    UART_TCC_VCP_INIT(inst);

DT_INST_FOREACH_STATUS_OKAY(UART_TCC_INSTANTIATE)
