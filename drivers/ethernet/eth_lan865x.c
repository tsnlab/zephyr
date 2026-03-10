/*
 * Copyright (c) 2023 DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan865x

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_lan865x, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>

#include <string.h>
#include <errno.h>

#include <zephyr/net/net_if.h>
#include <zephyr/drivers/ethernet/eth_lan865x.h>

#include "eth_lan865x_priv.h"

static int lan865x_default_config(const struct device *dev);
static void lan865x_rx_callback_handler(struct lan865x_data *ctx,
                                        struct net_pkt *pkt);

int eth_lan865x_mdio_c22_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			      uint16_t *data)
{
	struct lan865x_data *ctx = dev->data;

	return oa_tc6_mdio_read(ctx->tc6, prtad, regad, data);
}

int eth_lan865x_mdio_c22_write(const struct device *dev, uint8_t prtad, uint8_t regad,
			       uint16_t data)
{
	struct lan865x_data *ctx = dev->data;

	return oa_tc6_mdio_write(ctx->tc6, prtad, regad, data);
}

int eth_lan865x_mdio_c45_read(const struct device *dev, uint8_t prtad, uint8_t devad,
			      uint16_t regad, uint16_t *data)
{
	struct lan865x_data *ctx = dev->data;

	return oa_tc6_mdio_read_c45(ctx->tc6, prtad, devad, regad, data);
}

int eth_lan865x_mdio_c45_write(const struct device *dev, uint8_t prtad, uint8_t devad,
			       uint16_t regad, uint16_t data)
{
	struct lan865x_data *ctx = dev->data;

	return oa_tc6_mdio_write_c45(ctx->tc6, prtad, devad, regad, data);
}

static int lan865x_mac_rxtx_control(const struct device *dev, bool en)
{
	struct lan865x_data *ctx = dev->data;
	uint32_t ctl = 0;

	if (en) {
		ctl = LAN865x_MAC_NCR_TXEN | LAN865x_MAC_NCR_RXEN;
	}

	return oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_NCR, ctl);
}

static int lan865x_enable_sync(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	uint32_t val;
	int ret;

	ret = oa_tc6_reg_read(ctx->tc6, OA_CONFIG0, &val);
	if (ret) {
		return ret;
	}
	val |= OA_CONFIG0_SYNC | OA_CONFIG0_RFA_ZARFE;
	ret = oa_tc6_reg_write(ctx->tc6, OA_CONFIG0, val);
	if (ret) {
		return ret;
	}

	return lan865x_mac_rxtx_control(dev, LAN865x_MAC_TXRX_ON);
}

static void lan865x_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct lan865x_data *ctx = dev->data;
	int ret;

	ret = lan865x_enable_sync(dev);
	if (ret) {
		LOG_ERR("LAN865x sync enable failed: %d\n", ret);
		return;
	}

	net_if_set_link_addr(iface, ctx->mac_address, sizeof(ctx->mac_address), NET_LINK_ETHERNET);

	if (ctx->iface == NULL) {
		ctx->iface = iface;
	}

	ethernet_init(iface);

	net_eth_carrier_on(iface);
	ctx->iface_initialized = true;
}

static enum ethernet_hw_caps lan865x_port_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE | ETHERNET_PROMISC_MODE;
}

static int lan865x_gpio_reset(const struct device *dev);
static void lan865x_write_macaddress(const struct device *dev);
static int lan865x_set_config(const struct device *dev, enum ethernet_config_type type,
			      const struct ethernet_config *config)
{
	const struct lan865x_config *cfg = dev->config;
	struct lan865x_data *ctx = dev->data;
	struct phy_plca_cfg plca_cfg;

	if (type == ETHERNET_CONFIG_TYPE_PROMISC_MODE) {
		return oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_NCFGR, LAN865x_MAC_NCFGR_CAF);
	}

	if (type == ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		memcpy(ctx->mac_address, config->mac_address.addr, sizeof(ctx->mac_address));

		lan865x_write_macaddress(dev);

		return net_if_set_link_addr(ctx->iface, ctx->mac_address, sizeof(ctx->mac_address),
					    NET_LINK_ETHERNET);
	}

	if (type == ETHERNET_CONFIG_TYPE_T1S_PARAM) {
		if (config->t1s_param.type == ETHERNET_T1S_PARAM_TYPE_PLCA_CONFIG) {
			plca_cfg.enable = config->t1s_param.plca.enable;
			plca_cfg.node_id = config->t1s_param.plca.node_id;
			plca_cfg.node_count = config->t1s_param.plca.node_count;
			plca_cfg.burst_count = config->t1s_param.plca.burst_count;
			plca_cfg.burst_timer = config->t1s_param.plca.burst_timer;
			plca_cfg.to_timer = config->t1s_param.plca.to_timer;

			return phy_set_plca_cfg(cfg->phy, &plca_cfg);
		}
	}

	return -ENOTSUP;
}

#if defined(CONFIG_ETH_LAN865X_USE_IRQ)
static int lan865x_wait_for_reset(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	uint8_t i;

	/* Wait for end of LAN865x reset */
	for (i = 0; !ctx->reset && i < LAN865X_RESET_TIMEOUT; i++) {
		k_msleep(1);
	}

	if (i == LAN865X_RESET_TIMEOUT) {
		LOG_ERR("LAN865x reset timeout reached!");
		return -ENODEV;
	}
}
#else
static int lan865x_wait_for_reset(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	uint32_t sts, ftr;
	uint8_t i;
	int ret;

	/* Poll RESETC in OA_STATUS0 to detect end of reset in polling mode */
	for (i = 0; i < LAN865X_RESET_TIMEOUT; i++) {
		ret = oa_tc6_reg_read(tc6, OA_STATUS0, &sts);
		if (ret == 0 && (sts & OA_STATUS0_RESETC)) {
			/* Clear RESETC latch */
			(void)oa_tc6_reg_write(tc6, OA_STATUS0, sts);

			/* Apply default config right after reset completion */
			lan865x_default_config(dev);

			/* Mark reset done */
			ctx->reset = true;

			/*
			 * OA-T1S note: reading status/chunk may be required to deassert IRQ_N.
			 * Keep it even in polling mode for spec compliance.
			 */
			(void)oa_tc6_read_status(tc6, &ftr);

			return 0;
		}

		k_msleep(1);
	}

	LOG_ERR("LAN865x reset timeout reached!");
	return -ENODEV;
}
#endif

static int lan865x_gpio_reset(const struct device *dev)
{
	const struct lan865x_config *cfg = dev->config;
	struct lan865x_data *ctx = dev->data;

	LOG_ERR("LAN865x lan865x_gpio_reset!");

	ctx->reset = false;
	ctx->tc6->protected = false;

	/* Perform (GPIO based) HW reset */
	/* assert RESET_N low for 10 µs (5 µs min) */
	gpio_pin_set_dt(&cfg->reset, 1);
	 k_msleep(1);
	/* deassert - end of reset indicated by IRQ_N low  */
	gpio_pin_set_dt(&cfg->reset, 0);
	k_msleep(10);

	return lan865x_wait_for_reset(dev);
}

static int lan865x_check_spi(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	uint32_t val;
	int ret;

	LOG_DBG("LAN865x: lan865x_check_spi reading DEVID via SPI");
	ret = oa_tc6_reg_read(ctx->tc6, LAN865x_DEVID, &val);
	if (ret < 0) {
		LOG_ERR("LAN865x: oa_tc6_reg_read failed");
		return -ENODEV;
	}

	ctx->silicon_rev = val & LAN865X_REV_MASK;
	if (ctx->silicon_rev != 1 && ctx->silicon_rev != 2) {
		LOG_ERR("LAN865x: silicon_rev failed");
		return -ENODEV;
	}

	ctx->chip_id = (val >> 4) & 0xFFFF;
	if (ctx->chip_id != LAN8650_DEVID && ctx->chip_id != LAN8651_DEVID) {
		LOG_ERR("LAN865x: LAN8650_DEVID failed");
		return -ENODEV;
	}

	return ret;
}

static void lan865x_write_macaddress(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	uint8_t *mac = &ctx->mac_address[0];
	uint32_t val;

	/* SPEC_ADD2_BOTTOM */
	val = (mac[3] << 24) | (mac[2] << 16) | (mac[1] << 8) | mac[0];
	oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_SAB2, val);

	/* SPEC_ADD2_TOP */
	val = (mac[5] << 8) | mac[4];
	oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_SAT2, val);

	/*
	 * SPEC_ADD1_BOTTOM - setting unique lower MAC address, back off time is
	 * generated out of it.
	 */
	val = (mac[5] << 24) | (mac[4] << 16) | (mac[3] << 8) | mac[2];
	oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_SAB1, val);
	/* SPEC_ADD1_TOP - write top register too for activation */
	val = mac[1] << 8 | mac[0];
	oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_SAT1, val);
}

static int lan865x_set_specific_multicast_addr(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	uint32_t mac_h_hash = 0xffffffff;
	uint32_t mac_l_hash = 0xffffffff;
	int ret;

	/* Enable hash for all multicast addresses */
	ret = oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_HRT, mac_h_hash);
	if (ret) {
		return ret;
	}

	ret = oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_HRB, mac_l_hash);
	if (ret) {
		return ret;
	}

	return oa_tc6_reg_write(ctx->tc6, LAN865x_MAC_NCFGR, LAN865x_MAC_NCFGR_MTIHEN);
}

static int lan865x_default_config(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	int ret;

	/* Enable protected control RW */
	oa_tc6_set_protected_ctrl(ctx->tc6, true);

	ret = oa_tc6_reg_write(ctx->tc6, LAN865X_FIXUP_REG, LAN865X_FIXUP_VALUE);
	if (ret) {
		return ret;
	}

	lan865x_write_macaddress(dev);
	lan865x_set_specific_multicast_addr(dev);

	return 0;
}

static void lan865x_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct lan865x_data *ctx = CONTAINER_OF(cb, struct lan865x_data, gpio_int_callback);

	k_sem_give(&ctx->int_sem);
}

static void lan865x_dump_pkt(struct net_pkt *pkt)
{
	struct net_buf *buf;
	int offset = 0;

	printk("LAN865x RX dump (len=%u):\n", net_pkt_get_len(pkt));

	for (buf = pkt->buffer; buf; buf = buf->frags) {
		for (int i = 0; i < buf->len; i++) {
			printk("%02x ", buf->data[i]);
			if ((offset++ & 0x0f) == 0x0f) {
				printk("\n");
			}
		}
	}
	printk("\n");
}

static void lan865x_read_chunks(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	struct net_pkt *pkt;
	int ret;

	pkt = net_pkt_rx_alloc(K_MSEC(CONFIG_ETH_LAN865X_TIMEOUT));
	if (!pkt) {
		LOG_ERR("OA RX: Could not allocate packet!");
		return;
	}

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = oa_tc6_read_chunks(tc6, pkt);
	if (ret < 0) {
		eth_stats_update_errors_rx(ctx->iface);
		net_pkt_unref(pkt);
		k_sem_give(&ctx->tx_rx_sem);
		return;
	}

    /* application RX callback */
    lan865x_rx_callback_handler(ctx, pkt);

	/* Feed buffer frame to IP stack */
	ret = net_recv_data(ctx->iface, pkt);
	if (ret < 0) {
		LOG_ERR("OA RX: Could not process packet (%d)!", ret);
		net_pkt_unref(pkt);
	}
	k_sem_give(&ctx->tx_rx_sem);
}

static void lan865x_int_thread(const struct device *dev)
{
	struct lan865x_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	uint32_t sts, ftr;
	int ret;

	LOG_ERR("lan865x_int_thread int_thread tick: reset=%d rca=%u", ctx->reset, ctx->tc6->rca);

	while (true) {
#if defined(CONFIG_ETH_LAN865X_USE_IRQ)
		k_sem_take(&ctx->int_sem, K_FOREVER);
#else
		k_sleep(K_MSEC(CONFIG_ETH_LAN865X_POLL_INTERVAL_MS));
#endif
		if (!ctx->reset) {
			oa_tc6_reg_read(tc6, OA_STATUS0, &sts);
			if (sts & OA_STATUS0_RESETC) {
				oa_tc6_reg_write(tc6, OA_STATUS0, sts);

				lan865x_default_config(dev);

				ctx->reset = true;
				/*
				 * According to OA T1S standard - it is mandatory to
				 * read chunk of data to get the IRQ_N negated (deasserted).
				 */
				oa_tc6_read_status(tc6, &ftr);
				continue;
			}
		}

#if !defined(CONFIG_ETH_LAN865X_USE_IRQ)
		/*
		 * Polling mode: do NOT force a chunk read when no data is pending.
		 * First read status to refresh RCA, then only read chunks if RCA > 0.
		 */
		oa_tc6_read_status(tc6, &ftr);
		if (tc6->rca == 0) {
			continue;
		}
#endif
		/*
		 * The IRQ_N is asserted when RCA becomes > 0. As described in
		 * OPEN Alliance 10BASE-T1x standard it is deasserted when first
		 * data header is received by LAN865x.
		 *
		 * Hence, it is mandatory to ALWAYS read at least one data chunk!
		 */
		do {
			lan865x_read_chunks(dev);
#if !defined(CONFIG_ETH_LAN865X_USE_IRQ)
			/* Yield to avoid starving other threads in cooperative mode */
    		k_yield();
#endif
		} while (tc6->rca > 0);

		ret = oa_tc6_check_status(tc6);
		if (ret == -EIO) {
			lan865x_gpio_reset(dev);
		}
	}
}

static int lan865x_init(const struct device *dev)
{
	const struct lan865x_config *cfg = dev->config;
	struct lan865x_data *ctx = dev->data;
	int ret;

	LOG_ERR("LAN865x iface init called");

	__ASSERT(cfg->spi.config.frequency <= LAN865X_SPI_MAX_FREQUENCY,
		 "SPI frequency exceeds supported maximum\n");

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO device %s is not ready", cfg->interrupt.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO, %d", ret);
		return ret;
	}

	/* Check SPI communication after reset */
	ret = lan865x_check_spi(dev);
	if (ret < 0) {
		LOG_ERR("SPI communication not working, %d", ret);
		return ret;
	}

#if defined(CONFIG_ETH_LAN865X_USE_IRQ)
	/*
	 * Configure interrupt service routine for LAN865x IRQ
	 */
	ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt GPIO, %d", ret);
		return ret;
	}

	gpio_init_callback(&(ctx->gpio_int_callback), lan865x_int_callback,
			   BIT(cfg->interrupt.pin));

	ret = gpio_add_callback(cfg->interrupt.port, &ctx->gpio_int_callback);
	if (ret < 0) {
		LOG_ERR("Failed to add INT callback, %d", ret);
		return ret;
	}

	gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	/* Start interruption-poll thread */
	ctx->tid_int = k_thread_create(
		&ctx->thread, ctx->thread_stack, CONFIG_ETH_LAN865X_IRQ_THREAD_STACK_SIZE,
		(k_thread_entry_t)lan865x_int_thread, (void *)dev, NULL, NULL,
		K_PRIO_COOP(CONFIG_ETH_LAN865X_IRQ_THREAD_PRIO), 0, K_NO_WAIT);
	k_thread_name_set(ctx->tid_int, "lan865x_interrupt");
#else
	LOG_INF("LAN865x IRQ disabled, using polling mode");
	/* Start poll thread */
	// ctx->tid_int = k_thread_create(
	// 	&ctx->thread, ctx->thread_stack, CONFIG_ETH_LAN865X_IRQ_THREAD_STACK_SIZE,
	// 	(k_thread_entry_t)lan865x_int_thread, (void *)dev, NULL, NULL,
	// 	K_PRIO_PREEMPT(CONFIG_ETH_LAN865X_IRQ_THREAD_PRIO), 0, K_NO_WAIT);

	ctx->tid_int = k_thread_create(
		&ctx->thread, ctx->thread_stack, CONFIG_ETH_LAN865X_IRQ_THREAD_STACK_SIZE,
		(k_thread_entry_t)lan865x_int_thread, (void *)dev, NULL, NULL,
		K_PRIO_COOP(CONFIG_ETH_LAN865X_IRQ_THREAD_PRIO), 0, K_NO_WAIT);

	LOG_ERR("lan865x thread created: tid=%p stack=%p size=%u prio=%d",
        ctx->tid_int, ctx->thread_stack, CONFIG_ETH_LAN865X_IRQ_THREAD_STACK_SIZE,
        CONFIG_ETH_LAN865X_IRQ_THREAD_PRIO);
	k_thread_name_set(ctx->tid_int, "lan865x_poll");
#endif

	/* Perform HW reset - 'rst-gpios' required property set in DT */
	if (!gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("Reset GPIO device %s is not ready", cfg->reset.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO, %d", ret);
		return ret;
	}

	ret = net_eth_mac_load(&cfg->mac_cfg, ctx->mac_address);
	if (ret == -ENODATA) {
		LOG_DBG("No MAC address configured for %s", dev->name);
	} else if (ret < 0) {
		LOG_ERR("Failed to load MAC address (%d)", ret);
		return ret;
	}

	/* initialize RX callback */
	ctx->rx_cb = NULL;
	ctx->rx_cb_user_data = NULL;

    /* TEMP: skip hardware reset for bring-up */
    ctx->reset = true;
	return 0;

//	return lan865x_gpio_reset(dev);
}

int lan865x_register_rx_callback(const struct device *dev,
				 lan865x_rx_cb_t cb,
				 void *user_data)
{
	struct lan865x_data *ctx = dev->data;

	ctx->rx_cb = cb;
	ctx->rx_cb_user_data = user_data;

	return 0;
}

int lan865x_tx_frame(const struct device *dev, const uint8_t *data, size_t len)
{
    struct lan865x_data *ctx = dev->data;
    struct oa_tc6 *tc6 = ctx->tc6;
    struct net_pkt *pkt;
    uint32_t ftr = 0;
    int ret, sret;

    if (!ctx->iface) {
        LOG_ERR("LAN865x iface not initialized");
        return -ENODEV;
    }

    pkt = net_pkt_alloc_with_buffer(ctx->iface,
                                    len,
                                    AF_UNSPEC,
                                    0,
                                    K_MSEC(100));
    if (!pkt) {
        LOG_ERR("TX pkt alloc failed");
        return -ENOMEM;
    }

    ret = net_pkt_write(pkt, data, len);
    if (ret) {
        LOG_ERR("pkt write failed %d", ret);
        net_pkt_unref(pkt);
        return ret;
    }

    net_pkt_cursor_init(pkt);

    k_sem_take(&ctx->tx_rx_sem, K_FOREVER);

    sret = oa_tc6_read_status(tc6, &ftr);
    LOG_ERR("TX pre-status: sret=%d ftr=0x%08x sync=%u txc=%u rca=%u protected=%d",
            sret, ftr, tc6->sync, tc6->txc, tc6->rca, tc6->protected);

    LOG_INF("LAN865x TX frame len=%d", len);

    ret = oa_tc6_send_chunks(tc6, pkt);

    LOG_ERR("TX result: ret=%d sync=%u txc=%u rca=%u",
            ret, tc6->sync, tc6->txc, tc6->rca);

    k_sem_give(&ctx->tx_rx_sem);

    net_pkt_unref(pkt);

    return ret;
}

static void lan865x_rx_callback_handler(struct lan865x_data *ctx,
                                         struct net_pkt *pkt)
{
    struct net_pkt *clone;
    struct net_buf *frag;

    if (!ctx->rx_cb) {
        return;
    }

    clone = net_pkt_clone(pkt, K_NO_WAIT);
    if (!clone) {
        LOG_WRN("LAN865x RX: pkt clone failed");
        return;
    }

    frag = clone->buffer;

    while (frag) {

        ctx->rx_cb(frag->data,
                   frag->len,
                   ctx->rx_cb_user_data);

        frag = frag->frags;
    }

    net_pkt_unref(clone);
}

static int lan865x_port_send(const struct device *dev, struct net_pkt *pkt)
{
	LOG_ERR("LAN865x TX packet len=%d", net_pkt_get_len(pkt));

	struct lan865x_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	int ret;

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = oa_tc6_send_chunks(tc6, pkt);

	/* Check if rca > 0 during half-duplex TX transmission */
	if (tc6->rca > 0) {
		k_sem_give(&ctx->int_sem);
	}

	k_sem_give(&ctx->tx_rx_sem);
	if (ret < 0) {
		LOG_ERR("TX transmission error, %d", ret);
		eth_stats_update_errors_tx(net_pkt_iface(pkt));
		return ret;
	}

	return 0;
}

const struct device *lan865x_get_phy(const struct device *dev)
{
	const struct lan865x_config *cfg = dev->config;

	return cfg->phy;
}

static const struct ethernet_api lan865x_api_func = {
	.iface_api.init = lan865x_iface_init,
	.get_capabilities = lan865x_port_get_capabilities,
	.set_config = lan865x_set_config,
	.send = lan865x_port_send,
	.get_phy = lan865x_get_phy,
};

#define LAN865X_DEFINE(inst)                                                                       \
	static const struct lan865x_config lan865x_config_##inst = {                               \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                                \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                               \
		.reset = GPIO_DT_SPEC_INST_GET(inst, rst_gpios),                                   \
		.phy = DEVICE_DT_GET(                                                              \
			DT_CHILD(DT_INST_CHILD(inst, lan865x_mdio), ethernet_phy_##inst)),         \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(inst),                                  \
	};                                                                                         \
                                                                                                   \
	struct oa_tc6 oa_tc6_##inst = {                                                            \
		.cps = 64, .protected = 0, .spi = &lan865x_config_##inst.spi};                     \
	static struct lan865x_data lan865x_data_##inst = {                                         \
		.tx_rx_sem = Z_SEM_INITIALIZER((lan865x_data_##inst).tx_rx_sem, 1, 1),             \
		.int_sem = Z_SEM_INITIALIZER((lan865x_data_##inst).int_sem, 0, 1),                 \
		.tc6 = &oa_tc6_##inst};                                                            \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, lan865x_init, NULL, &lan865x_data_##inst,              \
				      &lan865x_config_##inst, CONFIG_ETH_LAN865X_INIT_PRIORITY,    \
				      &lan865x_api_func, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(LAN865X_DEFINE);
