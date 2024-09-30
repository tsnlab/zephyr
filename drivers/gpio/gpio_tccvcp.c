/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tcc_tccvcp_gpio

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#define GIO_DATA         0x00
#define GIO_OUT_EN       0x04
#define GIO_OUT_DATA_OR  0x08
#define GIO_OUT_DATA_BIC 0x0C
#define GIO_OUT_DATA_XOR 0x10
#define GIO_IN_EN        0x24
#define GIO_IODIR        0x08

#define DEV_CFG(dev)  ((const struct gpio_tccvcp_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_tccvcp_data *)(dev)->data)

struct gpio_tccvcp_config {
	struct gpio_driver_config common;

	DEVICE_MMIO_NAMED_ROM(reg_base);
	mem_addr_t offset;
};

struct gpio_tccvcp_data {
	struct gpio_driver_data common;

	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;
};

static int gpio_tccvcp_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_tccvcp_data *data = port->data;

	if (flags & (GPIO_SINGLE_ENDED | GPIO_PULL_UP | GPIO_PULL_DOWN)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INPUT) {
		sys_set_bit(data->base + GIO_IN_EN, pin);
	} else if (flags & GPIO_OUTPUT) {
		sys_set_bit(data->base + GIO_OUT_EN, pin);
	}

	return SAL_RET_SUCCESS;
}

static int gpio_tccvcp_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	struct gpio_tccvcp_data *data = port->data;

	*value = sys_read32(data->base + GIO_DATA);

	return SAL_RET_SUCCESS;
}

static int gpio_tccvcp_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_write32(mask, data->base + GIO_OUT_DATA_BIC);
	sys_write32((value & mask), data->base + GIO_OUT_DATA_OR);

	return SAL_RET_SUCCESS;
}

static int gpio_tccvcp_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_write32(pins, data->base + GIO_OUT_DATA_OR);

	return SAL_RET_SUCCESS;
}

static int gpio_tccvcp_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_write32(pins, data->base + GIO_OUT_DATA_BIC);

	return SAL_RET_SUCCESS;
}

static int gpio_tccvcp_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;
	uint32_t reg_data;

	reg_data = sys_read32(data->base + GIO_DATA);
	if (reg_data & pins) { /* 1 -> 0 */
		sys_write32(pins, data->base + GIO_OUT_DATA_BIC);
	} else { /* 0 -> 1 */
		sys_write32(pins, data->base + GIO_OUT_DATA_OR);
	}

	return SAL_RET_SUCCESS;
}

static const struct gpio_driver_api gpio_tccvcp_api = {
	.pin_configure = gpio_tccvcp_pin_configure,
	.port_get_raw = gpio_tccvcp_port_get_raw,
	.port_set_masked_raw = gpio_tccvcp_port_set_masked_raw,
	.port_set_bits_raw = gpio_tccvcp_port_set_bits_raw,
	.port_clear_bits_raw = gpio_tccvcp_port_clear_bits_raw,
	.port_toggle_bits = gpio_tccvcp_port_toggle_bits,
};

int gpio_tccvcp_init(const struct device *port)
{
	const struct gpio_tccvcp_config *config = port->config;
	struct gpio_tccvcp_data *data = port->data;

	DEVICE_MMIO_NAMED_MAP(port, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(port, reg_base) + config->offset;

	return 0;
}

#define GPIO_TCCVCP_INIT(n)                                                                        \
	static struct gpio_tccvcp_data gpio_tccvcp_data_##n;                                       \
                                                                                                   \
	static const struct gpio_tccvcp_config gpio_tccvcp_cfg_##n = {                             \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},                   \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_INST_PARENT(n)),                           \
		.offset = DT_INST_REG_ADDR(n),                                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_tccvcp_init, NULL, &gpio_tccvcp_data_##n,                    \
			      &gpio_tccvcp_cfg_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,       \
			      &gpio_tccvcp_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_TCCVCP_INIT)
