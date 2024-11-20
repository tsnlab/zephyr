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

int mfio_ch_cfg_flag[3] = {
	0,
};

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
		/* Disable the UART controller Bus clock */
		iClkBusId = (int32_t)CLOCK_IOBUS_UART0 + (int32_t)channel;
		(void)CLOCK_SetIobusPwdn(iClkBusId, TRUE);

		ret = UART_ClearGpio(channel);

		/* Disable the UART ch */
		sys_write32((uint32_t)NULL,
			    MCU_BSP_UART_BASE + (0x10000UL * (channel)) + UART_REG_CR);

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
	uart_param_t uart_pars;
    uint8_t uart_port;

    uart_port = ((UART_BASE_ADDR - 0xA0200000) / 0x10000);

	uart_pars.channel = uart_port;
	uart_pars.priority = GIC_PRIORITY_NO_MEAN;
	uart_pars.baud_rate = 115200;
	uart_pars.mode = UART_POLLING_MODE;
	uart_pars.cts_rts = UART_CTSRTS_OFF;
	//uart_pars.port_cfg = (uint8_t)(4U + uart_port);
	uart_pars.port_cfg = (uint8_t)(4U);
	uart_pars.fifo = DISABLE_FIFO, uart_pars.stop_bit = TWO_STOP_BIT_OFF;
	uart_pars.word_length = WORD_LEN_8;
	uart_pars.parity = PARITY_SPACE;
	uart_pars.callback_fn = NULL_PTR;

	(void)uart_close(uart_pars.channel);
	(void)UART_Open(&uart_pars);

	return 0;
}

int uart_tccvcp_poll_in(const struct device *dev, unsigned char *c)
{
	uint8_t ucCh;
	uint32_t data;
	uint32_t repeat = 0;

    ucCh = (uint8_t)((UART_BASE_ADDR - 0xA0200000) / 0x10000);

	if (ucCh >= UART_CH_MAX) {
		return -1;
	} else {
        while ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_RXFE) != 0UL)
        {
            if ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_RXFE) == 0UL)
            {
                break;
            }
            repeat++;
            if(repeat > 100) {
                return -1;
            }
        }

        data = UART_RegRead(ucCh, UART_REG_DR);
        *c = (unsigned char)(data & 0xFFUL);
	}

	return 0;
}

void uart_tccvcp_poll_out(const struct device *dev, unsigned char c)
{
	uint8_t ucCh;
	uint32_t repeat = 0;

    ucCh = (uint8_t)((UART_BASE_ADDR - 0xA0200000) / 0x10000);

	if (ucCh >= UART_CH_MAX) {
		return;
	} else {
        while ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_TXFF) != 0UL)
        {
            if ((UART_RegRead(ucCh, UART_REG_FR) & UART_FR_TXFF) == 0UL)
            {
                break;
            }
            repeat++;
            if(repeat > 100) {
                return;
            }
        }
		UART_RegWrite(ucCh, UART_REG_DR, c);
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
	uint8_t ucCh;
	uart_param_t uart_cfg;
	int32_t ret;

    ucCh = (uint8_t)((UART_BASE_ADDR - 0xA0200000) / 0x10000);

	uart_cfg.channel = ucCh;
	uart_cfg.priority = GIC_PRIORITY_NO_MEAN;
	uart_cfg.baud_rate = 115200;
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
	uint8_t ucCh;

    ucCh = (uint8_t)((UART_BASE_ADDR - 0xA0200000) / 0x10000);
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
