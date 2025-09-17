/*
 * Copyright (c) 2025 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT brcm_bcm2711_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_bcm2711);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/irq.h>
#include <zephyr/sys/byteorder.h>

#include "spi_context.h"

#define SPI_CS(base)   (base + 0x00)
#define SPI_FIFO(base) (base + 0x04)
#define SPI_CLK(base)  (base + 0x08)
#define SPI_DLEN(base) (base + 0x0C)
#define SPI_LTOH(base) (base + 0x10)
#define SPI_DC(base)   (base + 0x14)

#define SPI_CS_CPHA  BIT(2)
#define SPI_CS_CPOL  BIT(3)
#define SPI_CS_CSPOL BIT(6)
#define SPI_CS_TA    BIT(7)

#define SPI_CS_RXD BIT(17)
#define SPI_CS_TXD BIT(18)

#define DFS_4B 4
#define DFS_2B 2
#define DFS_1B 1

#define DEV_CFG(dev)  ((const struct spi_bcm2711_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct spi_bcm2711_data *const)(dev)->data)

struct spi_bcm2711_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);

	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec miso_gpio;
	struct gpio_dt_spec mosi_gpio;
	struct gpio_dt_spec cs_gpio;

	uint32_t clock_freq;
};

struct spi_bcm2711_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;

	struct spi_context ctx;
	uint8_t dfs;
};

static int spi_bcm2711_configure(const struct device *port, const struct spi_config *spi_cfg)
{
	const struct spi_bcm2711_config *config = port->config;
	struct spi_bcm2711_data *data = port->data;
	uint32_t word_size;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		return 0;
	}

	uint32_t clk = config->clock_freq / spi_cfg->frequency;
	sys_write32(clk, SPI_CLK(data->base));

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	}

	uint32_t spi_cs = sys_read32(SPI_CS(data->base));

	if (spi_cfg->operation & SPI_MODE_CPOL) {
		spi_cs |= SPI_CS_CPOL;
	} else {
		spi_cs &= ~SPI_CS_CPOL;
	}

	if (spi_cfg->operation & SPI_MODE_CPHA) {
		spi_cs |= SPI_CS_CPHA;
	} else {
		spi_cs &= ~SPI_CS_CPHA;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("Transfer LSB first mode is not supported");
		return -ENOTSUP;
	}

	word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size != 8 && word_size != 16 && word_size != 32) {
		LOG_ERR("Invalid word size: %d", word_size);
		return -EINVAL;
	}
	data->dfs = word_size / 8;

	if (spi_cfg->operation & SPI_HOLD_ON_CS) {
		LOG_ERR("Hold on CS is not supported");
		/* TODO: Only DMA can use this option */
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock on is not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) {
		spi_cs |= SPI_CS_CSPOL;
	} else {
		spi_cs &= ~SPI_CS_CSPOL;
	}

	sys_write32(spi_cs, SPI_CS(data->base));

	data->ctx.config = spi_cfg;

	return 0;
}

static int spi_bcm2711_xfer(const struct device *port)
{
	struct spi_bcm2711_data *data = port->data;

	uint32_t cs = sys_read32(SPI_CS(data->base));
	cs |= SPI_CS_TA;
	sys_write32(cs, SPI_CS(data->base));

	cs = sys_read32(SPI_CS(data->base));
	while (spi_context_tx_buf_on(&data->ctx) ||
	       (spi_context_rx_buf_on(&data->ctx) && (cs & SPI_CS_RXD))) {
		/* Tx */
		if (spi_context_tx_buf_on(&data->ctx) && (cs & SPI_CS_TXD)) {
			switch (data->dfs) {
			case DFS_4B:
				sys_write32(sys_get_be32(data->ctx.tx_buf), SPI_FIFO(data->base));
				break;
			case DFS_2B:
				sys_write32((uint32_t)sys_get_be16(data->ctx.tx_buf),
					    SPI_FIFO(data->base));
				break;
			case DFS_1B:
				sys_write32((uint32_t)*(uint8_t *)data->ctx.tx_buf,
					    SPI_FIFO(data->base));
				break;
			}
			spi_context_update_tx(&data->ctx, data->dfs, 1);
		}

		/* Rx */
		if (spi_context_rx_buf_on(&data->ctx) && (cs & SPI_CS_RXD)) {
			switch (data->dfs) {
			case DFS_4B:
				sys_put_be32(sys_read32(SPI_FIFO(data->base)), data->ctx.rx_buf);
				break;
			case DFS_2B:
				sys_put_be16(sys_read32(SPI_FIFO(data->base)), data->ctx.rx_buf);
				break;
			case DFS_1B:
				*(uint8_t *)data->ctx.rx_buf = sys_read32(SPI_FIFO(data->base));
				break;
			}
			spi_context_update_rx(&data->ctx, data->dfs, 1);
		}

		cs = sys_read32(SPI_CS(data->base));
	}

	return 0;
}

static int spi_bcm2711_transceive(const struct device *port, const struct spi_config *spi_cfg,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	struct spi_bcm2711_data *data = port->data;
	int ret = 0;

	spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);
	ret = spi_bcm2711_configure(port, spi_cfg);

	if (ret < 0) {
		goto end;
	}
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->dfs);
	ret = spi_bcm2711_xfer(port);
	spi_context_complete(&data->ctx, port, ret);

end:
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_bcm2711_release(const struct device *port, const struct spi_config *config)
{
	return -ENOTSUP;
}

static DEVICE_API(spi, spi_bcm2711_api) = {
	.transceive = spi_bcm2711_transceive,
	.release = spi_bcm2711_release,
};

static int spi_bcm2711_init(const struct device *port)
{
	const struct spi_bcm2711_config *config = port->config;
	struct spi_bcm2711_data *data = port->data;

	DEVICE_MMIO_NAMED_MAP(port, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(port, reg_base);

	if (config->clock_freq == 0) {
		LOG_ERR("Invalid clock frequency: %d", config->clock_freq);
		return -EINVAL;
	}

#define BCM2711_GPIO_BASE  0xfe200000 /* There's no API for this right now */
#define BCM2711_GPI0_FUNC0 0x4
	mm_reg_t gpio_base;
	device_map(&gpio_base, BCM2711_GPIO_BASE, 0x1000, K_MEM_CACHE_NONE);
	uint32_t fsel = sys_read32(gpio_base + 0x00);
	fsel |= (BCM2711_GPI0_FUNC0 << 24); /* PIN8 */
	fsel |= (BCM2711_GPI0_FUNC0 << 27); /* PIN9 */
	sys_write32(fsel, gpio_base + 0x00);
	fsel = sys_read32(gpio_base + 0x04);
	fsel |= (BCM2711_GPI0_FUNC0 << 0); /* PIN10 */
	fsel |= (BCM2711_GPI0_FUNC0 << 3); /* PIN11 */
	sys_write32(fsel, gpio_base + 0x04);
#undef BCM2711_GPI0_FUNC0
#undef BCM2711_GPIO_BASE

	/* TODO: This should be done by SPI_CONTEXT_INIT_* but somehow it's not working */
	/* TODO: Fix the issue and remove this */
	k_sem_init(&data->ctx.lock, 0, 1);
	k_sem_init(&data->ctx.sync, 0, 1);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define SPI_BCM2711_INIT(n)                                                                        \
	static const struct spi_bcm2711_config spi_bcm2711_cfg_##n = {                             \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                              \
		.clk_gpio = GPIO_DT_SPEC_INST_GET(n, clk_gpios),                                   \
		.miso_gpio = GPIO_DT_SPEC_INST_GET(n, miso_gpios),                                 \
		.mosi_gpio = GPIO_DT_SPEC_INST_GET(n, mosi_gpios),                                 \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                                     \
		.clock_freq = DT_PROP(DT_DRV_INST(n), clock_frequency),                            \
	};                                                                                         \
	static struct spi_bcm2711_data spi_bcm2711_data_##n = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_bcm2711_data_##n, ctx),                                  \
		SPI_CONTEXT_INIT_SYNC(spi_bcm2711_data_##n, ctx),                                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_bcm2711_init, NULL, &spi_bcm2711_data_##n,                    \
			      &spi_bcm2711_cfg_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,         \
			      &spi_bcm2711_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_BCM2711_INIT)
