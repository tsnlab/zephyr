/*
 * Copyright (c) 2023 DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ETH_LAN865X_PRIV_H__
#define ETH_LAN865X_PRIV_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/random/random.h>
#include <ethernet/eth_stats.h>

#include "oa_tc6.h"

/*
 * Compatibility fallback for Zephyr trees that do not yet provide
 * Ethernet MAC address configuration helpers.
 *
 * Newer Zephyr provides these from <zephyr/net/ethernet.h>:
 *   - enum net_eth_mac_type
 *   - struct net_eth_mac_config
 *   - net_eth_mac_load()
 *   - NET_ETH_MAC_DT_CONFIG_INIT()
 *   - NET_ETH_MAC_DT_INST_CONFIG_INIT()
 *
 * Keep this fallback local to the LAN865x driver so older trees can build
 * without modifying the common Ethernet core header.
 */
#ifndef NET_ETH_MAC_DT_INST_CONFIG_INIT

/** MAC address configuration types */
enum net_eth_mac_type {
	/** MAC address is handled by the driver */
	NET_ETH_MAC_DEFAULT = 0,

	/** A random MAC address is generated during initialization */
	NET_ETH_MAC_RANDOM,

	/** A static MAC address is provided in devicetree */
	NET_ETH_MAC_STATIC,
};

/** MAC address configuration */
struct net_eth_mac_config {
	enum net_eth_mac_type type;
	uint8_t addr[NET_ETH_ADDR_LEN];
	uint8_t addr_len;
};

/**
 * Load a MAC address from a MAC address configuration.
 *
 * This fallback supports the cases needed by LAN865x:
 *   - static MAC from local-mac-address
 *   - random MAC from zephyr,random-mac-address
 *   - optional random suffix using zephyr,mac-address-prefix
 */
static inline int net_eth_mac_load(const struct net_eth_mac_config *cfg,
				   uint8_t *mac_addr)
{
	if (cfg == NULL || mac_addr == NULL) {
		return -EINVAL;
	}

	if (cfg->type == NET_ETH_MAC_DEFAULT) {
		return -ENODATA;
	}

	if (cfg->addr_len > NET_ETH_ADDR_LEN) {
		return -EINVAL;
	}

	memset(mac_addr, 0, NET_ETH_ADDR_LEN);
	memcpy(mac_addr, cfg->addr, cfg->addr_len);

	if (cfg->type == NET_ETH_MAC_STATIC) {
		if (cfg->addr_len != NET_ETH_ADDR_LEN) {
			return -EINVAL;
		}

		return 0;
	}

	if (cfg->type == NET_ETH_MAC_RANDOM) {
		sys_rand_get(&mac_addr[cfg->addr_len],
			     NET_ETH_ADDR_LEN - cfg->addr_len);

		/* Clear group bit: unicast address */
		mac_addr[0] &= ~0x01;

		/* Set local administration bit */
		mac_addr[0] |= 0x02;

		return 0;
	}

	return -ENODATA;
}

#define Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_STATIC(node_id)		\
	{								\
		.type = NET_ETH_MAC_STATIC,				\
		.addr = DT_PROP(node_id, local_mac_address),		\
		.addr_len = DT_PROP_LEN(node_id, local_mac_address),	\
	}

#define Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_RANDOM(node_id)		\
	{								\
		.type = NET_ETH_MAC_RANDOM,				\
		.addr = DT_PROP_OR(node_id, zephyr_mac_address_prefix, {0}), \
		.addr_len = DT_PROP_LEN_OR(node_id, zephyr_mac_address_prefix, 0), \
	}

#define Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_DEFAULT(node_id)		\
	{								\
		.type = NET_ETH_MAC_DEFAULT,				\
	}

#define NET_ETH_MAC_DT_CONFIG_INIT(node_id)				\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, local_mac_address),	\
		    (Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_STATIC(node_id)), \
		    (COND_CODE_1(DT_PROP_OR(node_id, zephyr_random_mac_address, 0), \
				 (Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_RANDOM(node_id)), \
				 (Z_LAN865X_NET_ETH_MAC_DT_CONFIG_INIT_DEFAULT(node_id)))))

#define NET_ETH_MAC_DT_INST_CONFIG_INIT(inst)				\
	NET_ETH_MAC_DT_CONFIG_INIT(DT_DRV_INST(inst))

#endif /* NET_ETH_MAC_DT_INST_CONFIG_INIT */

/*
 * LAN865x RX callback prototype
 *
 * This callback is invoked when the LAN865x driver receives
 * Ethernet frame data from the device.
 *
 * The callback is executed in the driver RX thread context.
 *
 * @param data Pointer to received data fragment
 * @param len Length of the data fragment in bytes
 * @param user_data User-defined pointer provided during registration
 */
typedef void (*lan865x_rx_cb_t)(const uint8_t *data,
				size_t len,
				void *user_data);

#define LAN865X_SPI_MAX_FREQUENCY 25000000U
#define LAN865X_HW_BOOT_DELAY_MS  7
#define LAN8650_DEVID             0x8650
#define LAN8651_DEVID             0x8651
#define LAN865X_REV_MASK          GENMASK(3, 0)
#define LAN865X_RESET_TIMEOUT     10

/* Memory Map Sector (MMS) 1 (0x1) */
#define LAN865x_MAC_NCR          MMS_REG(0x1, 0x000)
#define LAN865x_MAC_NCR_TXEN     BIT(3)
#define LAN865x_MAC_NCR_RXEN     BIT(2)
#define LAN865x_MAC_NCFGR        MMS_REG(0x1, 0x001)
#define LAN865x_MAC_NCFGR_CAF    BIT(4)
#define LAN865x_MAC_NCFGR_MTIHEN BIT(6)
#define LAN865x_MAC_HRB          MMS_REG(0x1, 0x020)
#define LAN865x_MAC_HRT          MMS_REG(0x1, 0x021)
#define LAN865x_MAC_SAB1         MMS_REG(0x1, 0x022)
#define LAN865x_MAC_SAT1         MMS_REG(0x1, 0x023)
#define LAN865x_MAC_SAB2         MMS_REG(0x1, 0x024)
#define LAN865x_MAC_SAT2         MMS_REG(0x1, 0x025)

/* LAN8650/1 configuration fixup from AN1760 */
#define LAN865X_FIXUP_REG        MMS_REG(0x1, 0x077)
#define LAN865X_FIXUP_VALUE      0x0028

#define LAN865x_MAC_TXRX_ON  1
#define LAN865x_MAC_TXRX_OFF 0

/* Memory Map Sector (MMS) 10 (0xA) */
#define LAN865x_DEVID MMS_REG(0xA, 0x094)

struct lan865x_config {
	const struct device *phy;
	struct spi_dt_spec spi;
	struct gpio_dt_spec interrupt;
	struct gpio_dt_spec reset;
	struct net_eth_mac_config mac_cfg;
	int32_t timeout;

	/* MAC */
	bool tx_cut_through_mode; /* 1 - tx cut through, 0 - Store and forward */
	bool rx_cut_through_mode; /* 1 - rx cut through, 0 - Store and forward */
};

struct lan865x_data {
	struct net_if *iface;
	struct gpio_callback gpio_int_callback;
	struct k_sem tx_rx_sem;
	struct k_sem int_sem;
	struct oa_tc6 *tc6;
	uint16_t chip_id;
	uint8_t silicon_rev;
	uint8_t mac_address[6];
	bool iface_initialized;
	bool reset;

	/* RX callback support */
	lan865x_rx_cb_t rx_cb;
	void *rx_cb_user_data;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ETH_LAN865X_IRQ_THREAD_STACK_SIZE);
	struct k_thread thread;
	k_tid_t tid_int;
};

#endif /* ETH_LAN865X_PRIV_H__ */