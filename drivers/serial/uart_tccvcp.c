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

/* Register offsets within the UART device's register space */
#define XUARTPS_CR_OFFSET	0x0000U  /**< Control Register [8:0] */
#define XUARTPS_MR_OFFSET	0x0004U  /**< Mode Register [9:0] */
#define XUARTPS_IER_OFFSET	0x0008U  /**< Interrupt Enable [12:0] */
#define XUARTPS_IDR_OFFSET	0x000CU  /**< Interrupt Disable [12:0] */
#define XUARTPS_IMR_OFFSET	0x0010U  /**< Interrupt Mask [12:0] */
#define XUARTPS_ISR_OFFSET	0x0014U  /**< Interrupt Status [12:0]*/
#define XUARTPS_BAUDGEN_OFFSET	0x0018U  /**< Baud Rate Generator [15:0] */
#define XUARTPS_RXTOUT_OFFSET	0x001CU  /**< RX Timeout [7:0] */
#define XUARTPS_RXWM_OFFSET	0x0020U  /**< RX FIFO Trigger Level [5:0] */
#define XUARTPS_MODEMCR_OFFSET	0x0024U  /**< Modem Control [5:0] */
#define XUARTPS_MODEMSR_OFFSET	0x0028U  /**< Modem Status [8:0] */
#define XUARTPS_SR_OFFSET	0x002CU  /**< Channel Status [14:0] */
#define XUARTPS_FIFO_OFFSET	0x0030U  /**< FIFO [7:0] */
#define XUARTPS_BAUDDIV_OFFSET	0x0034U  /**< Baud Rate Divider [7:0] */
#define XUARTPS_FLOWDEL_OFFSET	0x0038U  /**< Flow Delay [5:0] */
#define XUARTPS_TXWM_OFFSET	0x0044U  /**< TX FIFO Trigger Level [5:0] */
#define XUARTPS_RXBS_OFFSET	0x0048U  /**< RX FIFO Byte Status [11:0] */

/* Control Register Bits Definition */
#define XUARTPS_CR_STOPBRK	0x00000100U  /**< Stop transmission of break */
#define XUARTPS_CR_STARTBRK	0x00000080U  /**< Set break */
#define XUARTPS_CR_TORST	0x00000040U  /**< RX timeout counter restart */
#define XUARTPS_CR_TX_DIS	0x00000020U  /**< TX disabled. */
#define XUARTPS_CR_TX_EN	0x00000010U  /**< TX enabled */
#define XUARTPS_CR_RX_DIS	0x00000008U  /**< RX disabled. */
#define XUARTPS_CR_RX_EN	0x00000004U  /**< RX enabled */
#define XUARTPS_CR_EN_DIS_MASK	0x0000003CU  /**< Enable/disable Mask */
#define XUARTPS_CR_TXRST	0x00000002U  /**< TX logic reset */
#define XUARTPS_CR_RXRST	0x00000001U  /**< RX logic reset */

/* Mode Register Bits Definition */
#define XUARTPS_MR_CCLK			0x00000400U /**< Input clock select */
#define XUARTPS_MR_CHMODE_R_LOOP	0x00000300U /**< Remote loopback mode */
#define XUARTPS_MR_CHMODE_L_LOOP	0x00000200U /**< Local loopback mode */
#define XUARTPS_MR_CHMODE_ECHO		0x00000100U /**< Auto echo mode */
#define XUARTPS_MR_CHMODE_NORM		0x00000000U /**< Normal mode */
#define XUARTPS_MR_CHMODE_SHIFT		8U  /**< Mode shift */
#define XUARTPS_MR_CHMODE_MASK		0x00000300U /**< Mode mask */
#define XUARTPS_MR_STOPMODE_2_BIT	0x00000080U /**< 2 stop bits */
#define XUARTPS_MR_STOPMODE_1_5_BIT	0x00000040U /**< 1.5 stop bits */
#define XUARTPS_MR_STOPMODE_1_BIT	0x00000000U /**< 1 stop bit */
#define XUARTPS_MR_STOPMODE_SHIFT	6U  /**< Stop bits shift */
#define XUARTPS_MR_STOPMODE_MASK	0x000000A0U /**< Stop bits mask */
#define XUARTPS_MR_PARITY_NONE		0x00000020U /**< No parity mode */
#define XUARTPS_MR_PARITY_MARK		0x00000018U /**< Mark parity mode */
#define XUARTPS_MR_PARITY_SPACE		0x00000010U /**< Space parity mode */
#define XUARTPS_MR_PARITY_ODD		0x00000008U /**< Odd parity mode */
#define XUARTPS_MR_PARITY_EVEN		0x00000000U /**< Even parity mode */
#define XUARTPS_MR_PARITY_SHIFT		3U  /**< Parity setting shift */
#define XUARTPS_MR_PARITY_MASK		0x00000038U /**< Parity mask */
#define XUARTPS_MR_CHARLEN_6_BIT	0x00000006U /**< 6 bits data */
#define XUARTPS_MR_CHARLEN_7_BIT	0x00000004U /**< 7 bits data */
#define XUARTPS_MR_CHARLEN_8_BIT	0x00000000U /**< 8 bits data */
#define XUARTPS_MR_CHARLEN_SHIFT	1U  /**< Data Length shift */
#define XUARTPS_MR_CHARLEN_MASK		0x00000006U /**< Data length mask */
#define XUARTPS_MR_CLKSEL		0x00000001U /**< Input clock select */

/* Interrupt Register Bits Definition */
#define XUARTPS_IXR_RBRK	0x00002000U /**< Rx FIFO break detect interrupt */
#define XUARTPS_IXR_TOVR	0x00001000U /**< Tx FIFO Overflow interrupt */
#define XUARTPS_IXR_TNFUL	0x00000800U /**< Tx FIFO Nearly Full interrupt */
#define XUARTPS_IXR_TTRIG	0x00000400U /**< Tx Trig interrupt */
#define XUARTPS_IXR_DMS		0x00000200U /**< Modem status change interrupt */
#define XUARTPS_IXR_TOUT	0x00000100U /**< Timeout error interrupt */
#define XUARTPS_IXR_PARITY	0x00000080U /**< Parity error interrupt */
#define XUARTPS_IXR_FRAMING	0x00000040U /**< Framing error interrupt */
#define XUARTPS_IXR_RXOVR	0x00000020U /**< Overrun error interrupt */
#define XUARTPS_IXR_TXFULL	0x00000010U /**< TX FIFO full interrupt. */
#define XUARTPS_IXR_TXEMPTY	0x00000008U /**< TX FIFO empty interrupt. */
#define XUARTPS_IXR_RXFULL	0x00000004U /**< RX FIFO full interrupt. */
#define XUARTPS_IXR_RXEMPTY	0x00000002U /**< RX FIFO empty interrupt. */
#define XUARTPS_IXR_RTRIG	0x00000001U /**< RX FIFO trigger interrupt. */
#define XUARTPS_IXR_MASK	0x00003FFFU /**< Valid bit mask */

/* Modem Control Register Bits Definition */
#define XUARTPS_MODEMCR_FCM_RTS_CTS	0x00000020 /**< RTS/CTS hardware flow control. */
#define XUARTPS_MODEMCR_FCM_NONE	0x00000000 /**< No hardware flow control. */
#define XUARTPS_MODEMCR_FCM_MASK	0x00000020 /**< Hardware flow control mask. */
#define XUARTPS_MODEMCR_RTS_SHIFT	1U         /**< RTS bit shift */
#define XUARTPS_MODEMCR_DTR_SHIFT	0U         /**< DTR bit shift */

/* Channel Status Register */
#define XUARTPS_SR_TNFUL	0x00004000U /**< TX FIFO Nearly Full Status */
#define XUARTPS_SR_TTRIG	0x00002000U /**< TX FIFO Trigger Status */
#define XUARTPS_SR_FLOWDEL	0x00001000U /**< RX FIFO fill over flow delay */
#define XUARTPS_SR_TACTIVE	0x00000800U /**< TX active */
#define XUARTPS_SR_RACTIVE	0x00000400U /**< RX active */
#define XUARTPS_SR_TXFULL	0x00000010U /**< TX FIFO full */
#define XUARTPS_SR_TXEMPTY	0x00000008U /**< TX FIFO empty */
#define XUARTPS_SR_RXFULL	0x00000004U /**< RX FIFO full */
#define XUARTPS_SR_RXEMPTY	0x00000002U /**< RX FIFO empty */
#define XUARTPS_SR_RTRIG	0x00000001U /**< RX FIFO fill over trigger */

/** Device configuration structure */
struct uart_tccvcp_dev_config {
	DEVICE_MMIO_ROM;
	uint32_t sys_clk_freq;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
	uint32_t baud_rate;
};

/** Device data structure */
struct uart_tccvcp_dev_data_t {
	DEVICE_MMIO_RAM;
	uint32_t parity;
	uint32_t stopbits;
	uint32_t databits;
	uint32_t flowctrl;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};

static void xlnx_ps_disable_uart(uintptr_t reg_base)
{
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_CR_OFFSET);

	reg_val &= (~XUARTPS_CR_EN_DIS_MASK);
	/* Set control register bits [5]: TX_DIS and [3]: RX_DIS */
	reg_val |= XUARTPS_CR_TX_DIS | XUARTPS_CR_RX_DIS;
	sys_write32(reg_val, reg_base + XUARTPS_CR_OFFSET);
}

static void xlnx_ps_enable_uart(uintptr_t reg_base)
{
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_CR_OFFSET);

	reg_val &= (~XUARTPS_CR_EN_DIS_MASK);
	/* Set control register bits [4]: TX_EN and [2]: RX_EN */
	reg_val |= XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN;
	sys_write32(reg_val, reg_base + XUARTPS_CR_OFFSET);
}

static void set_baudrate(const struct device *dev, uint32_t baud_rate)
{
	const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
	uint32_t baud = dev_cfg->baud_rate;
	uint32_t clk_freq = dev_cfg->sys_clk_freq;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t divisor, generator;

	/* Calculate divisor and baud rate generator value */
	if ((baud != 0) && (clk_freq != 0)) {
		/* Covering case where input clock is so slow */
		if (clk_freq < 1000000U && baud > 4800U) {
			baud = 4800;
		}

		for (divisor = 4; divisor < 255; divisor++) {
			uint32_t tmpbaud, bauderr;

			generator = clk_freq / (baud * (divisor + 1));
			if (generator < 2 || generator > 65535) {
				continue;
			}
			tmpbaud = clk_freq / (generator * (divisor + 1));

			if (baud > tmpbaud) {
				bauderr = baud - tmpbaud;
			} else {
				bauderr = tmpbaud - baud;
			}
			if (((bauderr * 100) / baud) < 3) {
				break;
			}
		}

		/*
		 * Set baud rate divisor and generator.
		 * -> This function is always called from a context in which
		 * the receiver/transmitter is disabled, the baud rate can
		 * be changed safely at this time.
		 */
		sys_write32(divisor, reg_base + XUARTPS_BAUDDIV_OFFSET);
		sys_write32(generator, reg_base + XUARTPS_BAUDGEN_OFFSET);
	}
}

static int uart_tccvcp_init(const struct device *dev)
{
	const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
	uint32_t reg_val;
#ifdef CONFIG_PINCTRL
	int err;
#endif

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	/* Disable RX/TX before changing any configuration data */
	xlnx_ps_disable_uart(reg_base);

#ifdef CONFIG_PINCTRL
	err = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}
#endif

	/* Set initial character length / start/stop bit / parity configuration */
	reg_val = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	reg_val &= (~(XUARTPS_MR_CHARLEN_MASK | XUARTPS_MR_STOPMODE_MASK |
		    XUARTPS_MR_PARITY_MASK));
	reg_val |= XUARTPS_MR_CHARLEN_8_BIT | XUARTPS_MR_STOPMODE_1_BIT |
		   XUARTPS_MR_PARITY_NONE;
	sys_write32(reg_val, reg_base + XUARTPS_MR_OFFSET);

	/* Set RX FIFO trigger at 1 data bytes. */
	sys_write32(0x01U, reg_base + XUARTPS_RXWM_OFFSET);

	/* Disable all interrupts, polling mode is default */
	sys_write32(XUARTPS_IXR_MASK, reg_base + XUARTPS_IDR_OFFSET);

	/* Set the baud rate */
	set_baudrate(dev, dev_cfg->baud_rate);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

	/* Clear any pending interrupt flags */
	sys_write32(XUARTPS_IXR_MASK, reg_base + XUARTPS_ISR_OFFSET);

	/* Attach to & unmask the corresponding interrupt vector */
	dev_cfg->irq_config_func(dev);

#endif

	xlnx_ps_enable_uart(reg_base);

	return 0;
}

static int uart_tccvcp_poll_in(const struct device *dev, unsigned char *c)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);

	if ((reg_val & XUARTPS_SR_RXEMPTY) == 0) {
		*c = (unsigned char)sys_read32(reg_base + XUARTPS_FIFO_OFFSET);
		return 0;
	} else {
		return -1;
	}
}

static void uart_tccvcp_poll_out(const struct device *dev, unsigned char c)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val;

	/* wait for transmitter to ready to accept a character */
	do {
		reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	} while ((reg_val & XUARTPS_SR_TXEMPTY) == 0);

	sys_write32((uint32_t)(c & 0xFF), reg_base + XUARTPS_FIFO_OFFSET);

	do {
		reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	} while ((reg_val & XUARTPS_SR_TXEMPTY) == 0);
}

static inline bool uart_tccvcp_cfg2ll_parity(
	uint32_t *mode_reg,
	enum uart_config_parity parity)
{
	/*
	 * Translate the new parity configuration to the mode register's
	 * bits [5..3] (PAR):
	 *  000b : even
	 *  001b : odd
	 *  010b : space
	 *  011b : mark
	 *  1xxb : none
	 */

	switch (parity) {
	default:
	case UART_CFG_PARITY_EVEN:
		*mode_reg |= XUARTPS_MR_PARITY_EVEN;
		break;
	case UART_CFG_PARITY_ODD:
		*mode_reg |= XUARTPS_MR_PARITY_ODD;
		break;
	case UART_CFG_PARITY_SPACE:
		*mode_reg |= XUARTPS_MR_PARITY_SPACE;
		break;
	case UART_CFG_PARITY_MARK:
		*mode_reg |= XUARTPS_MR_PARITY_MARK;
		break;
	case UART_CFG_PARITY_NONE:
		*mode_reg |= XUARTPS_MR_PARITY_NONE;
		break;
	}

	return true;
}

static inline bool uart_tccvcp_cfg2ll_stopbits(
	uint32_t *mode_reg,
	enum uart_config_stop_bits stopbits)
{
	/*
	 * Translate the new stop bit configuration to the mode register's
	 * bits [7..6] (NBSTOP):
	 *  00b : 1 stop bit
	 *  01b : 1.5 stop bits
	 *  10b : 2 stop bits
	 *  11b : reserved
	 */

	switch (stopbits) {
	case UART_CFG_STOP_BITS_0_5:
		/* Controller doesn't support 0.5 stop bits */
		return false;
	default:
	case UART_CFG_STOP_BITS_1:
		*mode_reg |= XUARTPS_MR_STOPMODE_1_BIT;
		break;
	case UART_CFG_STOP_BITS_1_5:
		*mode_reg |= XUARTPS_MR_STOPMODE_1_5_BIT;
		break;
	case UART_CFG_STOP_BITS_2:
		*mode_reg |= XUARTPS_MR_STOPMODE_2_BIT;
		break;
	}

	return true;
}

static inline bool uart_tccvcp_cfg2ll_databits(
	uint32_t *mode_reg,
	enum uart_config_data_bits databits)
{
	/*
	 * Translate the new data bit configuration to the mode register's
	 * bits [2..1] (CHRL):
	 *  0xb : 8 data bits
	 *  10b : 7 data bits
	 *  11b : 6 data bits
	 */

	switch (databits) {
	case UART_CFG_DATA_BITS_5:
	case UART_CFG_DATA_BITS_9:
		/* Controller doesn't support 5 or 9 data bits */
		return false;
	default:
	case UART_CFG_DATA_BITS_8:
		*mode_reg |= XUARTPS_MR_CHARLEN_8_BIT;
		break;
	case UART_CFG_DATA_BITS_7:
		*mode_reg |= XUARTPS_MR_CHARLEN_7_BIT;
		break;
	case UART_CFG_DATA_BITS_6:
		*mode_reg |= XUARTPS_MR_CHARLEN_6_BIT;
		break;
	}

	return true;
}

static inline bool uart_tccvcp_cfg2ll_hwctrl(
	uint32_t *modemcr_reg,
	enum uart_config_flow_control hwctrl)
{
	/*
	 * Translate the new flow control configuration to the modem
	 * control register's bit [5] (FCM):
	 *  0b : no flow control
	 *  1b : RTS/CTS
	 */

	if (hwctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		*modemcr_reg |= XUARTPS_MODEMCR_FCM_RTS_CTS;
	} else if (hwctrl ==  UART_CFG_FLOW_CTRL_NONE) {
		*modemcr_reg |= XUARTPS_MODEMCR_FCM_NONE;
	} else {
		/* Only no flow control or RTS/CTS is supported. */
		return false;
	}

	return true;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_tccvcp_configure(const struct device *dev,
				  const struct uart_config *cfg)
{
	struct uart_tccvcp_dev_config *dev_cfg =
	(struct uart_tccvcp_dev_config *)dev->config;

	uintptr_t reg_base   = DEVICE_MMIO_GET(dev);
	uint32_t mode_reg    = 0;
	uint32_t modemcr_reg = 0;

	/* Read the current mode register & modem control register values */
	mode_reg    = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	modemcr_reg = sys_read32(reg_base + XUARTPS_MODEMCR_OFFSET);

	/* Mask out all items that might be re-configured */
	mode_reg    &= (~XUARTPS_MR_PARITY_MASK);
	mode_reg    &= (~XUARTPS_MR_STOPMODE_MASK);
	mode_reg    &= (~XUARTPS_MR_CHARLEN_MASK);
	modemcr_reg &= (~XUARTPS_MODEMCR_FCM_MASK);

	/* Assemble the updated registers, validity checks contained within */
	if ((!uart_tccvcp_cfg2ll_parity(&mode_reg, cfg->parity)) ||
		(!uart_tccvcp_cfg2ll_stopbits(&mode_reg, cfg->stop_bits)) ||
		(!uart_tccvcp_cfg2ll_databits(&mode_reg, cfg->data_bits)) ||
		(!uart_tccvcp_cfg2ll_hwctrl(&modemcr_reg, cfg->flow_ctrl))) {
		return -ENOTSUP;
	}

	/* Disable the controller before modifying any config registers */
	xlnx_ps_disable_uart(reg_base);

	/* Set the baud rate */
	set_baudrate(dev, cfg->baudrate);
	dev_cfg->baud_rate = cfg->baudrate;

	/* Write the two control registers */
	sys_write32(mode_reg,    reg_base + XUARTPS_MR_OFFSET);
	sys_write32(modemcr_reg, reg_base + XUARTPS_MODEMCR_OFFSET);

	/* Re-enable the controller */
	xlnx_ps_enable_uart(reg_base);

	return 0;
};
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static inline enum uart_config_parity uart_tccvcp_ll2cfg_parity(
	uint32_t mode_reg)
{
	/*
	 * Obtain the current parity configuration from the mode register's
	 * bits [5..3] (PAR):
	 *  000b : even -> reset value
	 *  001b : odd
	 *  010b : space
	 *  011b : mark
	 *  1xxb : none
	 */

	switch ((mode_reg & XUARTPS_MR_PARITY_MASK)) {
	case XUARTPS_MR_PARITY_EVEN:
	default:
		return UART_CFG_PARITY_EVEN;
	case XUARTPS_MR_PARITY_ODD:
		return UART_CFG_PARITY_ODD;
	case XUARTPS_MR_PARITY_SPACE:
		return UART_CFG_PARITY_SPACE;
	case XUARTPS_MR_PARITY_MARK:
		return UART_CFG_PARITY_MARK;
	case XUARTPS_MR_PARITY_NONE:
		return UART_CFG_PARITY_NONE;
	}
}

static inline enum uart_config_stop_bits uart_tccvcp_ll2cfg_stopbits(
	uint32_t mode_reg)
{
	/*
	 * Obtain the current stop bit configuration from the mode register's
	 * bits [7..6] (NBSTOP):
	 *  00b : 1 stop bit -> reset value
	 *  01b : 1.5 stop bits
	 *  10b : 2 stop bits
	 *  11b : reserved
	 */

	switch ((mode_reg & XUARTPS_MR_STOPMODE_MASK)) {
	case XUARTPS_MR_STOPMODE_1_BIT:
	default:
		return UART_CFG_STOP_BITS_1;
	case XUARTPS_MR_STOPMODE_1_5_BIT:
		return UART_CFG_STOP_BITS_1_5;
	case XUARTPS_MR_STOPMODE_2_BIT:
		return UART_CFG_STOP_BITS_2;
	}
}

static inline enum uart_config_data_bits uart_tccvcp_ll2cfg_databits(
	uint32_t mode_reg)
{
	/*
	 * Obtain the current data bit configuration from the mode register's
	 * bits [2..1] (CHRL):
	 *  0xb : 8 data bits -> reset value
	 *  10b : 7 data bits
	 *  11b : 6 data bits
	 */

	switch ((mode_reg & XUARTPS_MR_CHARLEN_MASK)) {
	case XUARTPS_MR_CHARLEN_8_BIT:
	default:
		return UART_CFG_DATA_BITS_8;
	case XUARTPS_MR_CHARLEN_7_BIT:
		return UART_CFG_DATA_BITS_7;
	case XUARTPS_MR_CHARLEN_6_BIT:
		return UART_CFG_DATA_BITS_6;
	}
}

static inline enum uart_config_flow_control uart_tccvcp_ll2cfg_hwctrl(
	uint32_t modemcr_reg)
{
	/*
	 * Obtain the current flow control configuration from the modem
	 * control register's bit [5] (FCM):
	 *  0b : no flow control -> reset value
	 *  1b : RTS/CTS
	 */

	if ((modemcr_reg & XUARTPS_MODEMCR_FCM_MASK)
		== XUARTPS_MODEMCR_FCM_RTS_CTS) {
		return UART_CFG_FLOW_CTRL_RTS_CTS;
	}

	return UART_CFG_FLOW_CTRL_NONE;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_tccvcp_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	const struct uart_tccvcp_dev_config *dev_cfg = dev->config;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	/*
	 * Read the Mode & Modem control registers - they contain
	 * the current data / stop bit and parity settings (Mode
	 * Register) and the current flow control setting (Modem
	 * Control register).
	 */
	uint32_t mode_reg    = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	uint32_t modemcr_reg = sys_read32(reg_base + XUARTPS_MODEMCR_OFFSET);

	cfg->baudrate  = dev_cfg->baud_rate;
	cfg->parity    = uart_tccvcp_ll2cfg_parity(mode_reg);
	cfg->stop_bits = uart_tccvcp_ll2cfg_stopbits(mode_reg);
	cfg->data_bits = uart_tccvcp_ll2cfg_databits(mode_reg);
	cfg->flow_ctrl = uart_tccvcp_ll2cfg_hwctrl(modemcr_reg);

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#if CONFIG_UART_INTERRUPT_DRIVEN

static int uart_tccvcp_fifo_fill(const struct device *dev,
				  const uint8_t *tx_data,
				  int size)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t data_iter = 0;

	sys_write32(XUARTPS_IXR_TXEMPTY, reg_base + XUARTPS_IDR_OFFSET);
	while (size--) {
		while ((sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_TXFULL) != 0) {
		}
		sys_write32((uint32_t)tx_data[data_iter++], reg_base + XUARTPS_FIFO_OFFSET);
	}
	sys_write32(XUARTPS_IXR_TXEMPTY, reg_base + XUARTPS_IER_OFFSET);

	return data_iter;
}

static int uart_tccvcp_fifo_read(const struct device *dev, uint8_t *rx_data,
				  const int size)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	int inum = 0;

	while (inum < size && (reg_val & XUARTPS_SR_RXEMPTY) == 0) {
		rx_data[inum] = (uint8_t)sys_read32(reg_base
				+ XUARTPS_FIFO_OFFSET);
		inum++;
		reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	}

	return inum;
}

static void uart_tccvcp_irq_tx_enable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(
		(XUARTPS_IXR_TTRIG | XUARTPS_IXR_TXEMPTY),
		reg_base + XUARTPS_IER_OFFSET);
}

static void uart_tccvcp_irq_tx_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(
		(XUARTPS_IXR_TTRIG | XUARTPS_IXR_TXEMPTY),
		reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_tccvcp_irq_tx_ready(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);

	if ((reg_val & (XUARTPS_SR_TTRIG | XUARTPS_SR_TXEMPTY)) == 0) {
		return 0;
	} else {
		return 1;
	}
}

static int uart_tccvcp_irq_tx_complete(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);

	if ((reg_val & XUARTPS_SR_TXEMPTY) == 0) {
		return 0;
	} else {
		return 1;
	}
}

static void uart_tccvcp_irq_rx_enable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_RTRIG, reg_base + XUARTPS_IER_OFFSET);
}

static void uart_tccvcp_irq_rx_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_RTRIG, reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_tccvcp_irq_rx_ready(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_ISR_OFFSET);

	if ((reg_val & XUARTPS_IXR_RTRIG) == 0) {
		return 0;
	} else {
		sys_write32(XUARTPS_IXR_RTRIG, reg_base + XUARTPS_ISR_OFFSET);
		return 1;
	}
}

static void uart_tccvcp_irq_err_enable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(
		  XUARTPS_IXR_TOVR    /* [12] Transmitter FIFO Overflow */
		| XUARTPS_IXR_TOUT    /* [8]  Receiver Timerout */
		| XUARTPS_IXR_PARITY  /* [7]  Parity Error */
		| XUARTPS_IXR_FRAMING /* [6]  Receiver Framing Error */
		| XUARTPS_IXR_RXOVR,  /* [5]  Receiver Overflow Error */
	    reg_base + XUARTPS_IER_OFFSET);
}

static void uart_tccvcp_irq_err_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(
		  XUARTPS_IXR_TOVR    /* [12] Transmitter FIFO Overflow */
		| XUARTPS_IXR_TOUT    /* [8]  Receiver Timerout */
		| XUARTPS_IXR_PARITY  /* [7]  Parity Error */
		| XUARTPS_IXR_FRAMING /* [6]  Receiver Framing Error */
		| XUARTPS_IXR_RXOVR,  /* [5]  Receiver Overflow Error */
	    reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_tccvcp_irq_is_pending(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_imr = sys_read32(reg_base + XUARTPS_IMR_OFFSET);
	uint32_t reg_isr = sys_read32(reg_base + XUARTPS_ISR_OFFSET);

	if ((reg_imr & reg_isr) != 0) {
		return 1;
	} else {
		return 0;
	}
}

static int uart_tccvcp_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void uart_tccvcp_irq_callback_set(const struct device *dev,
					    uart_irq_callback_user_data_t cb,
					    void *cb_data)
{
	struct uart_tccvcp_dev_data_t *dev_data = dev->data;

	dev_data->user_cb = cb;
	dev_data->user_data = cb_data;
}

static void uart_tccvcp_isr(const struct device *dev)
{
	const struct uart_tccvcp_dev_data_t *data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_tccvcp_driver_api = {
	.poll_in = uart_tccvcp_poll_in,
	.poll_out = uart_tccvcp_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_tccvcp_configure,
	.config_get = uart_tccvcp_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_tccvcp_fifo_fill,
	.fifo_read = uart_tccvcp_fifo_read,
	.irq_tx_enable = uart_tccvcp_irq_tx_enable,
	.irq_tx_disable = uart_tccvcp_irq_tx_disable,
	.irq_tx_ready = uart_tccvcp_irq_tx_ready,
	.irq_tx_complete = uart_tccvcp_irq_tx_complete,
	.irq_rx_enable = uart_tccvcp_irq_rx_enable,
	.irq_rx_disable = uart_tccvcp_irq_rx_disable,
	.irq_rx_ready = uart_tccvcp_irq_rx_ready,
	.irq_err_enable = uart_tccvcp_irq_err_enable,
	.irq_err_disable = uart_tccvcp_irq_err_disable,
	.irq_is_pending = uart_tccvcp_irq_is_pending,
	.irq_update = uart_tccvcp_irq_update,
	.irq_callback_set = uart_tccvcp_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define UART_TCC_VCP_IRQ_CONF_FUNC_SET(port) \
	.irq_config_func = uart_tccvcp_irq_config_##port,

#define UART_TCC_VCP_IRQ_CONF_FUNC(port) \
static void uart_tccvcp_irq_config_##port(const struct device *dev) \
{ \
	IRQ_CONNECT(DT_INST_IRQN(port), \
	DT_INST_IRQ(port, priority), \
	uart_tccvcp_isr, DEVICE_DT_INST_GET(port), \
	0); \
	irq_enable(DT_INST_IRQN(port)); \
}

#else

#define UART_TCC_VCP_IRQ_CONF_FUNC_SET(port)
#define UART_TCC_VCP_IRQ_CONF_FUNC(port)

#endif /*CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_TCC_VCP_DEV_DATA(port) \
static struct uart_tccvcp_dev_data_t uart_tccvcp_dev_data_##port

#if CONFIG_PINCTRL
#define UART_TCC_VCP_PINCTRL_DEFINE(port) PINCTRL_DT_INST_DEFINE(port);
#define UART_TCC_VCP_PINCTRL_INIT(port) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(port),
#else
#define UART_TCC_VCP_PINCTRL_DEFINE(port)
#define UART_TCC_VCP_PINCTRL_INIT(port)
#endif /* CONFIG_PINCTRL */

#define UART_TCC_VCP_DEV_CFG(port) \
static struct uart_tccvcp_dev_config uart_tccvcp_dev_cfg_##port = { \
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(port)), \
	.sys_clk_freq = DT_INST_PROP(port, clock_frequency), \
	.baud_rate = DT_INST_PROP(port, current_speed), \
	UART_TCC_VCP_IRQ_CONF_FUNC_SET(port) \
	UART_TCC_VCP_PINCTRL_INIT(port) \
}

#define UART_TCC_VCP_INIT(port) \
DEVICE_DT_INST_DEFINE(port, \
	uart_tccvcp_init, \
	NULL, \
	&uart_tccvcp_dev_data_##port, \
	&uart_tccvcp_dev_cfg_##port, \
	PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
	&uart_tccvcp_driver_api)

#define UART_TCC_INSTANTIATE(inst)		\
	UART_TCC_VCP_PINCTRL_DEFINE(inst)	\
	UART_TCC_VCP_IRQ_CONF_FUNC(inst);	\
	UART_TCC_VCP_DEV_DATA(inst);		\
	UART_TCC_VCP_DEV_CFG(inst);		\
	UART_TCC_VCP_INIT(inst);

DT_INST_FOREACH_STATUS_OKAY(UART_TCC_INSTANTIATE)
