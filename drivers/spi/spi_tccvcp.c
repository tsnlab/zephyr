/*
 * Copyright (c) 2025 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_tccvcp, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT tcc_tccvcp_spi

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

struct spi_tccvcp_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);
};

struct spi_tccvcp_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
};


#define DEV_CFG(dev)  ((const struct spi_tccvcp_config *)(dev)->config)
#define DEV_DATA(dev) ((struct spi_tccvcp_data *)(dev)->data)

static int spi_tccvcp_transceive(const struct device *port,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	// Ignore parameters, Do ping-pong here

	mm_reg_t spi_base = DEVICE_MMIO_NAMED_GET(port, reg_base);

	uint32_t stat = sys_read32(spi_base + 0x04);
	uint32_t wbvcnt = (stat >> 24) & 0x1f;

	while (wbvcnt == 0) {
		stat = sys_read32(spi_base + 0x04);
		wbvcnt = (stat >> 24) & 0x1f;
	}

	sys_write32(0xab, spi_base + 0x00);  // Send ping

	stat = sys_read32(spi_base + 0x04);
	uint32_t rbvcnt = (stat >> 16) & 0x1f;

	while (rbvcnt == 0) {
		stat = sys_read32(spi_base + 0x04);
		rbvcnt = (stat >> 16) & 0x1f;
	}

	printk("%u\n", sys_read32(spi_base + 0x00));  // Receive pong

	return 0;
}

static int spi_tccvcp_release(const struct device *port,
			      const struct spi_config *config)
{
	return 0;
}

static DEVICE_API(spi, spi_tccvcp_api) = {
	.transceive = spi_tccvcp_transceive,
	.release = spi_tccvcp_release,
};

static int spi_tccvcp_init(const struct device *port)
{
	const struct spi_tccvcp_config *config = port->config;
	struct spi_tccvcp_data *data = port->data;

	DEVICE_MMIO_NAMED_MAP(port, reg_base, K_MEM_CACHE_NONE);

	// TEMP
	const uintptr_t GPIO_BASE = 0xA0F22000;
	const size_t GPIO_SIZE = 0x1000;
	// mm_reg_t gpio_base;
	// device_map(&gpio_base, GPIO_BASE, GPIO_SIZE, K_MEM_CACHE_NONE);

	// Cortex-R does not have an MMU
	// Must check if this works
	sys_write32(0x11110000, GPIO_BASE + 0x70);  // Function
	sys_write32(0x70, GPIO_BASE + 0x44);        // Output enable
	sys_write32(0x80, GPIO_BASE + 0x64);        // Input enable

	sys_set_bits(DEVICE_MMIO_NAMED_GET(port, reg_base) + 0xc, 0xf << 8);  // BPW

	return 0;
}

#define SPI_TCCVCP_INIT(n)                                                                           \
	static struct spi_tccvcp_data spi_tccvcp_data_##n;                                             \
                                                                                                   \
	static const struct spi_tccvcp_config spi_tccvcp_cfg_##n = {                                   \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_tccvcp_init, NULL, &spi_tccvcp_data_##n, &spi_tccvcp_cfg_##n,       \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_tccvcp_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_TCCVCP_INIT)
