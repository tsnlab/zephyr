/*
 * Copyright (c) 2025 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/arch/arm/cortex_a_r/sys_io.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_tccvcp, CONFIG_SPI_LOG_LEVEL);

#define DT_DRV_COMPAT tcc_tccvcp_spi

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_tccvcp.h>

#include "spi_context.h"

#define SPI_DATA(base)    (base + 0x00)
#define SPI_STAT(base)    (base + 0x04)
#define SPI_MODE(base)    (base + 0x0c)
#define SPI_CTRL(base)    (base + 0x10)
#define SPI_EVTCTRL(base) (base + 0x14)

#define SPI_STAT_RTH_LSB 0
#define SPI_STAT_RTH_MASK (0x1 << SPI_STAT_RTH_LSB)
#define SPI_STAT_RTH(stat) (((stat & SPI_STAT_RTH_MASK) >> SPI_STAT_RTH_LSB) & 0x1)

#define SPI_STAT_WTH_LSB 1
#define SPI_STAT_WTH_MASK (0x1 << SPI_STAT_WTH_LSB)
#define SPI_STAT_WTH(stat) (((stat & SPI_STAT_WTH_MASK) >> SPI_STAT_WTH_LSB) & 0x1)

#define SPI_STAT_WBVCNT_LSB 24
#define SPI_STAT_WBVCNT_MASK (0x1f << SPI_STAT_WBVCNT_LSB)
#define SPI_STAT_WBVCNT(stat) (((stat & SPI_STAT_WBVCNT_MASK) >> SPI_STAT_WBVCNT_LSB) & 0x1f)

#define SPI_STAT_RBVCNT_LSB 16
#define SPI_STAT_RBVCNT_MASK (0x1f << SPI_STAT_RBVCNT_LSB)
#define SPI_STAT_RBVCNT(stat) (((stat & SPI_STAT_RBVCNT_MASK) >> SPI_STAT_RBVCNT_LSB) & 0x1f)

#define SPI_MODE_MD_LSB 0
#define SPI_MODE_MD_MASK (0x3 << SPI_MODE_MD_LSB)
#define SPI_MODE_SLV_LSB 2
#define SPI_MODE_SLV_MASK (0x1 << SPI_MODE_SLV_LSB)
#define SPI_MODE_EN_LSB 3
#define SPI_MODE_EN_MASK (0x1 << SPI_MODE_EN_LSB)
#define SPI_MODE_CTF_LSB 4
#define SPI_MODE_CTF_MASK (0x1 << SPI_MODE_CTF_LSB)
#define SPI_MODE_SDO_LSB 5
#define SPI_MODE_SDO_MASK (0x1 << SPI_MODE_SDO_LSB)
#define SPI_MODE_LB_LSB 6
#define SPI_MODE_LB_MASK (0x1 << SPI_MODE_LB_LSB)
#define SPI_MODE_SD_LSB 7
#define SPI_MODE_SD_MASK (0x1 << SPI_MODE_SD_LSB)
#define SPI_MODE_BPW_LSB 8
#define SPI_MODE_BPW_MASK (0x1f << SPI_MODE_BPW_LSB)

#define SPI_MODE_PCK_LSB 16
#define SPI_MODE_PCK_MASK (0x1 << SPI_MODE_PCK_LSB)
#define SPI_MODE_PRD_LSB 17
#define SPI_MODE_PRD_MASK (0x1 << SPI_MODE_PRD_LSB)
#define SPI_MODE_PWD_LSB 18
#define SPI_MODE_PWD_MASK (0x1 << SPI_MODE_PWD_LSB)
#define SPI_MODE_PCD_LSB 19
#define SPI_MODE_PCD_MASK (0x1 << SPI_MODE_PCD_LSB)
#define SPI_MODE_PCS_LSB 20
#define SPI_MODE_PCS_MASK (0x1 << SPI_MODE_PCS_LSB)

#define SPI_MODE_DIVLDV_LSB 24
#define SPI_MODE_DIVLDV_MASK (0xff << SPI_MODE_DIVLDV_LSB)

struct spi_tccvcp_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);

	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec miso_gpio;
	struct gpio_dt_spec mosi_gpio;
	struct gpio_dt_spec cs_gpio;

	uint32_t clock_freq;
};

struct spi_tccvcp_data {
	mm_reg_t reg_base;
	struct spi_context ctx;
	uint8_t dfs;
};


#define DEV_CFG(dev)  ((const struct spi_tccvcp_config *)(dev)->config)
#define DEV_DATA(dev) ((struct spi_tccvcp_data *)(dev)->data)

static int spi_tccvcp_configure(const struct device *port, const struct spi_config *spi_cfg)
{
	const struct spi_tccvcp_config *config = port->config;
	struct spi_tccvcp_data *data = port->data;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		return 0;
	}

	uint32_t spi_mode = sys_read32(SPI_MODE(data->reg_base));

	/* TODO: Use another peripheral clock? XIN cannot support frequency higher than 6MHz */
	int32_t divldv_ = ((int32_t)config->clock_freq / (spi_cfg->frequency * 2)) - 1;
	if (divldv_ < 0 || divldv_ > 0xff) {
		LOG_ERR("Invalid frequency: %d", spi_cfg->frequency);
		return -EINVAL;
	}

	uint32_t divldv = divldv_ & 0xff;
	spi_mode &= ~SPI_MODE_DIVLDV_MASK;
	spi_mode |= divldv << SPI_MODE_DIVLDV_LSB;

	LOG_DBG("Given frequency: %u, Actual frequency: %u", spi_cfg->frequency, config->clock_freq / ((divldv + 1) * 2));

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	}

	spi_mode &= ~SPI_MODE_SLV_MASK;

	if (spi_cfg->operation & SPI_MODE_CPOL) {
		spi_mode |= SPI_MODE_PCK_MASK;
	} else {
		spi_mode &= ~SPI_MODE_PCK_MASK;
	}

	if (spi_cfg->operation & SPI_MODE_CPHA) {
		spi_mode |= SPI_MODE_PRD_MASK;
		spi_mode |= SPI_MODE_PWD_MASK;
	} else {
		spi_mode &= ~SPI_MODE_PRD_MASK;
		spi_mode &= ~SPI_MODE_PWD_MASK;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP) {
		spi_mode |= SPI_MODE_LB_MASK;
	} else {
		spi_mode &= ~SPI_MODE_LB_MASK;
	}

	if (spi_cfg->operation & SPI_TRANSFER_MSB) {
		spi_mode |= SPI_MODE_SD_MASK;
	} else {
		spi_mode &= ~SPI_MODE_SD_MASK;
	}

	uint32_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size != 8 && word_size != 16 && word_size != 32) {
		LOG_ERR("Invalid word size: %d", word_size);
		return -EINVAL;
	}

	uint32_t bpw = word_size - 1;
	spi_mode &= ~SPI_MODE_BPW_MASK;
	spi_mode |= bpw << SPI_MODE_BPW_LSB;

	data->dfs = word_size / 8;

	if (spi_cfg->operation & SPI_HOLD_ON_CS) {
		spi_mode |= SPI_MODE_CTF_MASK;
	} else {
		spi_mode &= ~SPI_MODE_CTF_MASK;
	}

	if (spi_cfg->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock on is not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) {
		spi_mode |= SPI_MODE_PCS_MASK;
	} else {
		spi_mode &= ~SPI_MODE_PCS_MASK;
	}

	sys_write32(spi_mode, SPI_MODE(data->reg_base));

	data->ctx.config = spi_cfg;

	return 0;
}

static int spi_tccvcp_xfer(const struct device *port) {
	struct spi_tccvcp_data *data = port->data;
	volatile uint32_t stat, wth, rth, spi_data;

	stat = sys_read32(SPI_STAT(data->reg_base));
	wth = SPI_STAT_WTH(stat);  /* wth == 1 means Write FIFO valid entry count(wbvcnt) is under threshold */
	rth = SPI_STAT_RTH(stat);  /* rth == 1 means Read FIFO valid entry count(rbvcnt) is over threshold */
	while (spi_context_tx_buf_on(&data->ctx) || (spi_context_rx_buf_on(&data->ctx) && rth)) {
		/* Tx */
		if (spi_context_tx_buf_on(&data->ctx) && wth) {
			switch (data->dfs) {
			case 4:
				spi_data = sys_get_be32(data->ctx.tx_buf);
				break;
			case 2:
				spi_data = sys_get_be16(data->ctx.tx_buf);
				break;
			case 1:
				spi_data = *(uint8_t*)data->ctx.tx_buf;
				break;
			default:
				LOG_ERR("Unsupported data size: %d", data->dfs * 8);
				return -EINVAL;
			}
			sys_write32(spi_data, SPI_DATA(data->reg_base));
			spi_context_update_tx(&data->ctx, data->dfs, 1);
		}

		/* Rx */
		if (spi_context_rx_buf_on(&data->ctx) && rth) {
			spi_data = sys_read32(SPI_DATA(data->reg_base));
			switch (data->dfs) {
			case 4:
				sys_put_be32(spi_data, data->ctx.rx_buf);
				break;
			case 2:
				sys_put_be16(spi_data, data->ctx.rx_buf);
				break;
			case 1:
				*(uint8_t*)data->ctx.rx_buf = spi_data;
				break;
			default:
				LOG_ERR("Unsupported data size: %d", data->dfs * 8);
				return -EINVAL;
			}
			spi_context_update_rx(&data->ctx, data->dfs, 1);
		}

		stat = sys_read32(SPI_STAT(data->reg_base));
		wth = SPI_STAT_WTH(stat);
		rth = SPI_STAT_RTH(stat);
		
		/* TODO: Communicating with RPi needs this */
		/* TODO: This needs to be tested with other devices */
		volatile int iii = 100000;
		while (iii--) { arch_nop(); }
	}

	return 0;
}

static int spi_tccvcp_transceive(const struct device *port,
				 const struct spi_config *spi_cfg,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct spi_tccvcp_data *data = port->data;
	int ret = 0;

	spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);
	if ((ret = spi_tccvcp_configure(port, spi_cfg)) < 0) {
		goto end;
	}
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->dfs);
	ret = spi_tccvcp_xfer(port);
	spi_context_complete(&data->ctx, port, ret);

end:
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_tccvcp_release(const struct device *port,
			      const struct spi_config *config)
{
	return -ENOTSUP;
}

static DEVICE_API(spi, spi_tccvcp_api) = {
	.transceive = spi_tccvcp_transceive,
	.release = spi_tccvcp_release,
};

static int spi_tccvcp_init(const struct device *port)
{
	const struct spi_tccvcp_config *config = port->config;
	struct spi_tccvcp_data *data = port->data;
	data->reg_base = DEVICE_MMIO_NAMED_GET(port, reg_base);

	if (config->clock_freq == 0) {
		LOG_ERR("Invalid clock frequency: %d", config->clock_freq);
		return -EINVAL;
	}

	/* TODO: This should be done by SPI_CONTEXT_INIT_* but somehow it's not working */
	/* TODO: Fix the issue and remove this */
	k_sem_init(&data->ctx.lock, 0, 1);
	k_sem_init(&data->ctx.sync, 0, 1);

	/* Configure GPIO Pins for SPI */

	/* Set the pins' function to SPI */
	vcp_gpio_config(GPIO_PORT_B | config->clk_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->miso_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->mosi_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->cs_gpio.pin, GPIO_FUNC(1UL));

	/* Output pins */
	gpio_pin_configure_dt(&config->clk_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->mosi_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT);

	/* Input pin */
	gpio_pin_configure_dt(&config->miso_gpio, GPIO_INPUT);

	/* Configure SPI settings */
	uint32_t spi_mode = sys_read32(SPI_MODE(data->reg_base));

	spi_mode &= ~SPI_MODE_MD_MASK;   /* Other values than 0 are reserved */
	spi_mode |= SPI_MODE_EN_MASK;    /* Enable SPI */

	sys_write32(spi_mode, SPI_MODE(data->reg_base));

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define SPI_TCCVCP_INIT(n)                                                        \
	static const struct spi_tccvcp_config spi_tccvcp_cfg_##n = {              \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),             \
		.clk_gpio = GPIO_DT_SPEC_INST_GET(n, clk_gpios),                  \
		.miso_gpio = GPIO_DT_SPEC_INST_GET(n, miso_gpios),                \
		.mosi_gpio = GPIO_DT_SPEC_INST_GET(n, mosi_gpios),                \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                    \
		.clock_freq = DT_PROP(DT_DRV_INST(n), clock_frequency),           \
	};                                                                        \
	static struct spi_tccvcp_data spi_tccvcp_data_##n = {                     \
		SPI_CONTEXT_INIT_LOCK(spi_tccvcp_data_##n, ctx),                  \
		SPI_CONTEXT_INIT_SYNC(spi_tccvcp_data_##n, ctx),                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)                             \
	};                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_tccvcp_init, NULL, &spi_tccvcp_data_##n, &spi_tccvcp_cfg_##n,       \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_tccvcp_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_TCCVCP_INIT)
