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

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_tccvcp.h>

#define SPI_DATA(base)    (base + 0x00)
#define SPI_STAT(base)    (base + 0x04)
#define SPI_MODE(base)    (base + 0x0c)
#define SPI_CTRL(base)    (base + 0x10)
#define SPI_EVTCTRL(base) (base + 0x14)

#define SPI_MODE_MD_LSB 0
#define SPI_MODE_MD_MASK (0x3 << SPI_MODE_MD_LSB)
#define SPI_MODE_SLV_LSB 2
#define SPI_MODE_SLV_MASK (0x1 << SPI_MODE_SLV_LSB)
#define SPI_MODE_EN_LSB 3
#define SPI_MODE_EN_MASK (0x1 << SPI_MODE_EN_LSB)

#define SPI_MODE_BPW_LSB 8
#define SPI_MODE_BPW_MASK (0x1f << SPI_MODE_BPW_LSB)

struct spi_tccvcp_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);

	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec miso_gpio;
	struct gpio_dt_spec mosi_gpio;
	struct gpio_dt_spec cs_gpio;
};

struct spi_tccvcp_data {
	mm_reg_t reg_base;
};


#define DEV_CFG(dev)  ((const struct spi_tccvcp_config *)(dev)->config)
#define DEV_DATA(dev) ((struct spi_tccvcp_data *)(dev)->data)

static int spi_tccvcp_transceive(const struct device *port,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
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
	data->reg_base = DEVICE_MMIO_NAMED_GET(port, reg_base);

	// Configure GPIO Pins for SPI

	// Set the pins' function to SPI
	vcp_gpio_config(GPIO_PORT_B | config->clk_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->miso_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->mosi_gpio.pin, GPIO_FUNC(1UL));
	vcp_gpio_config(GPIO_PORT_B | config->cs_gpio.pin, GPIO_FUNC(1UL));

	// Output pins
	gpio_pin_configure_dt(&config->clk_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->mosi_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT);

	// Input pin
	gpio_pin_configure_dt(&config->miso_gpio, GPIO_INPUT);


	// Configure SPI settings
	uint32_t spi_mode = sys_read32(SPI_MODE(data->reg_base));

	spi_mode &= ~SPI_MODE_MD_MASK;   // Other values than 0 are reserved
	spi_mode |= SPI_MODE_EN_MASK;    // Enable SPI

	sys_write32(spi_mode, SPI_MODE(data->reg_base));

	spi_tccvcp_transceive(port, NULL, NULL, NULL);

	return 0;
}

#define SPI_TCCVCP_INIT(n)                                                                           \
	static struct spi_tccvcp_data spi_tccvcp_data_##n;                                             \
                                                                                                   \
	static const struct spi_tccvcp_config spi_tccvcp_cfg_##n = {                                   \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                           \
		.clk_gpio = GPIO_DT_SPEC_INST_GET(n, clk_gpios),                  \
		.miso_gpio = GPIO_DT_SPEC_INST_GET(n, miso_gpios),                \
		.mosi_gpio = GPIO_DT_SPEC_INST_GET(n, mosi_gpios),                \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_tccvcp_init, NULL, &spi_tccvcp_data_##n, &spi_tccvcp_cfg_##n,       \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_tccvcp_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_TCCVCP_INIT)
