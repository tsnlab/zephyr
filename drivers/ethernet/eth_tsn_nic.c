/*
 * Copyright (c) 2024 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* TODO: Better name */
#define DT_DRV_COMPAT tsnlab_tsn_nic_eth

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_tsn_nic, LOG_LEVEL_ERR);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/posix/posix_types.h>
#include <zephyr/posix/pthread.h>

#include "eth.h"

#define ETH_ALEN 6

/* TODO: use sizeof() after implementing tx/rx */
#define BUFFER_SIZE (1500 + 60) /* Ethernet MTU + TSN Metadata */

#define DESC_MAGIC 0xAD4B0000UL

#define DESC_STOPPED   (1UL << 0)
#define DESC_COMPLETED (1UL << 1)
#define DESC_EOP       (1UL << 4)

#define LS_BYTE_MASK 0x000000FFUL

#define PCI_DMA_H(addr) ((addr >> 16) >> 16)
#define PCI_DMA_L(addr) (addr & 0xffffffffUL)

#define DMA_ENGINE_START 16268831
#define DMA_ENGINE_STOP  16268830

#define RX_METADATA_SIZE (sizeof(struct rx_metadata))
#define CRC_LEN          4

/* Temporary macros: need to be deleted later */
#define RX_ENGINE_REG_ADDR 0x1b08001000
#define RX_SGDMA_REG_ADDR  0x1b08005000
#define RX_REGS_SIZE       0x1000

struct dma_tsn_nic_engine_regs {
	uint32_t identifier;
	uint32_t control;
	uint32_t control_w1s;
	uint32_t control_w1c;
	uint32_t reserved_1[12]; /* padding */

	uint32_t status;
	uint32_t status_rc;
	uint32_t completed_desc_count;
	uint32_t alignments;
	uint32_t reserved_2[14]; /* padding */

	uint32_t poll_mode_wb_lo;
	uint32_t poll_mode_wb_hi;
	uint32_t interrupt_enable_mask;
	uint32_t interrupt_enable_mask_w1s;
	uint32_t interrupt_enable_mask_w1c;
	uint32_t reserved_3[9]; /* padding */

	uint32_t perf_ctrl;
	uint32_t perf_cyc_lo;
	uint32_t perf_cyc_hi;
	uint32_t perf_dat_lo;
	uint32_t perf_dat_hi;
	uint32_t perf_pnd_lo;
	uint32_t perf_pnd_hi;
} __packed;

struct dma_tsn_nic_engine_sgdma_regs {
	uint32_t identifier;
	uint32_t reserved_1[31]; /* padding */

	/* bus address to first descriptor in Root Complex Memory */
	uint32_t first_desc_lo;
	uint32_t first_desc_hi;
	/* number of adjacent descriptors at first_desc */
	uint32_t first_desc_adjacent;
	uint32_t credits;
} __packed;

struct dma_tsn_nic_desc {
	uint32_t control;
	uint32_t bytes;
	uint32_t src_addr_lo;
	uint32_t src_addr_hi;
	uint32_t dst_addr_lo;
	uint32_t dst_addr_hi;
	uint32_t next_lo;
	uint32_t next_hi;
};

struct dma_tsn_nic_result {
	uint32_t status;
	uint32_t length;
	uint32_t reserved_1[6]; /* padding */
};

struct rx_metadata {
	uint64_t timestamp;
	uint16_t frame_length;
} __packed __attribute__((scalar_storage_order("big-endian")));

struct eth_tsn_nic_config {
	const struct device *pci_dev;
	const struct device *dma_dev;
};

struct eth_tsn_nic_data {
	struct net_if *iface;

	uint8_t mac_addr[NET_ETH_ADDR_LEN];

	pthread_spinlock_t tx_lock;
	pthread_spinlock_t rx_lock;

	struct dma_tsn_nic_desc tx_desc;
	struct dma_tsn_nic_desc rx_desc;

	/* TODO: Maybe this needs to be allocated dynamically */
	uint8_t rx_buffer[BUFFER_SIZE];

	struct dma_tsn_nic_result res;

	struct k_work rx_work;

	bool has_pkt; /* TODO: This is for test only */
};

static void eth_tsn_nic_isr(const struct device *port)
{
	/* TODO: Implement interrupts */
	ARG_UNUSED(port);
}

static void rx_desc_set(struct dma_tsn_nic_desc *desc, uintptr_t addr, uint32_t len)
{
	uint32_t control_field;
	uint32_t control;

	desc->control = sys_cpu_to_le32(DESC_MAGIC);
	control_field = DESC_STOPPED;
	control_field |= DESC_EOP;
	control_field |= DESC_COMPLETED;
	control = sys_le32_to_cpu(desc->control & ~(LS_BYTE_MASK));
	control |= control_field;
	desc->control = sys_cpu_to_le32(control);

	desc->dst_addr_lo = sys_cpu_to_le32(PCI_DMA_L(addr));
	desc->dst_addr_hi = sys_cpu_to_le32(PCI_DMA_H(addr));
	desc->bytes = sys_cpu_to_le32(len);
}

static void eth_tsn_nic_rx(struct k_work *item)
{
	struct eth_tsn_nic_data *data = CONTAINER_OF(item, struct eth_tsn_nic_data, rx_work);
	struct net_pkt *pkt;
	struct net_ptp_time timestamp;
	size_t pkt_len;
	int ret;

	pthread_spin_lock(&data->rx_lock);

	if (!data->has_pkt) { /* Check if there is a packet to receive */
		goto done;
	}

	/* TODO: disable interrupts */

	/* pkt_len = data->res.length - RX_METADATA_SIZE - CRC_LEN */;
	pkt_len = data->res.length;
	if (pkt_len < 0) {
		goto done;
	}

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		/* TODO: stats */
		printk("alloc failed\n");
		goto done;
	}

	ret = net_pkt_write(pkt, data->rx_buffer, pkt_len);
	/* ret = net_pkt_write(pkt, data->rx_buffer + RX_METADATA_SIZE, pkt_len); */
	if (ret != 0) {
		net_pkt_unref(pkt);
		printk("write failed\n");
		goto done;
	}

	ret = net_recv_data(data->iface, pkt);
	if (ret != 0) {
		net_pkt_unref(pkt);
		printk("recv failed\n");
		goto done;
	}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	/* Not sure this is how it's supposed to be used as there are not much examples */
	if (net_pkt_is_rx_timestamping(pkt)) {
		/* TODO: Get HW timestamp */
		net_pkt_set_timestamp(pkt, &timestamp);
	}
#else
	ARG_UNUSED(timestamp);
#endif /* CONFIG_NET_PKT_TIMESTMP */

done:
	/* TODO: enable interrupts */
	data->has_pkt = false; /* TODO: This is for test only */

	pthread_spin_unlock(&data->rx_lock);
}

static void eth_tsn_nic_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_tsn_nic_data *data = dev->data;

	if (data->iface == NULL) {
		data->iface = iface;
	}

	net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);
	ethernet_init(iface);
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_tsn_nic_get_stats(const struct device *dev)
{
	/* TODO: sw-257 (Misc. APIs) */
	return NULL;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static int eth_tsn_nic_start(const struct device *dev)
{
	struct eth_tsn_nic_data *data = dev->data;
	mm_reg_t rx_regs, rx_sgdma_regs;

	data->rx_desc.src_addr_lo = sys_cpu_to_le32(PCI_DMA_L((uintptr_t)&data->res));
	data->rx_desc.src_addr_hi = sys_cpu_to_le32(PCI_DMA_H((uintptr_t)&data->res));

	rx_desc_set(&data->rx_desc, (uintptr_t)&data->rx_buffer, BUFFER_SIZE);

	/**
	 * TODO: Find out how to move this to dma driver
	 * or how to access dma registers from here
	 */
	device_map(&rx_regs, RX_ENGINE_REG_ADDR, RX_REGS_SIZE, K_MEM_CACHE_NONE);
	device_map(&rx_sgdma_regs, RX_SGDMA_REG_ADDR, RX_REGS_SIZE, K_MEM_CACHE_NONE);

	pthread_spin_lock(&data->rx_lock);

	/* TODO: It seems the board is not reading the descriptor properly */
	sys_read32(rx_regs + offsetof(struct dma_tsn_nic_engine_regs, status_rc)); /* Clear reg */
	sys_write32(sys_cpu_to_le32(PCI_DMA_L((uintptr_t)&data->rx_desc)),
		    rx_regs + offsetof(struct dma_tsn_nic_engine_sgdma_regs, first_desc_lo));
	sys_write32(sys_cpu_to_le32(PCI_DMA_H((uintptr_t)&data->rx_desc)),
		    rx_regs + offsetof(struct dma_tsn_nic_engine_sgdma_regs, first_desc_hi));
	sys_write32(DMA_ENGINE_START, rx_regs + offsetof(struct dma_tsn_nic_engine_regs, control));

	pthread_spin_unlock(&data->rx_lock);

	return 0;
}

static int eth_tsn_nic_stop(const struct device *dev)
{
	/* TODO: sw-257 (Misc. APIs) */
	return -ENOTSUP;
}

static enum ethernet_hw_caps eth_tsn_nic_get_capabilities(const struct device *dev)
{
	/* TODO: sw-257 (Misc. APIs) */
	return -ENOTSUP;
}

static int eth_tsn_nic_set_config(const struct device *dev, enum ethernet_config_type type,
				  const struct ethernet_config *config)
{
	/* TODO: sw-295 (QoS) */
	return -ENOTSUP;
}

static int eth_tsn_nic_get_config(const struct device *dev, enum ethernet_config_type type,
				  struct ethernet_config *config)
{
	/* TODO: sw-295 (QoS) */
	return -ENOTSUP;
}

#if defined(CONFIG_NET_VLAN)
static int eth_tsn_nic_vlan_setup(const struct device *dev, struct net_if *iface, uint16_t tag,
				  bool enable)
{
	/* TODO: sw-257 (Misc. APIs) or a new issue */
	return -ENOTSUP;
}
#endif /* CONFIG_NET_VLAN */

#if defined(CONFIG_PTP_CLOCK)
static const struct device *eth_tsn_nic_get_ptp_clock(const struct device *dev)
{
	/* TODO: sw-290 (PTP) */
	return NULL;
}
#endif /* CONFIG_PTP_CLOCK */

static const struct device *eth_tsn_nic_get_phy(const struct device *dev)
{
	/* TODO: sw-257 (Misc. APIs) */
	/* This might not be needed at all */
	return NULL;
}

static int eth_tsn_nic_send(const struct device *dev, struct net_pkt *pkt)
{
	/* TODO: This is for test only */
	struct eth_tsn_nic_data *data = dev->data;
	size_t len;

	len = net_pkt_get_len(pkt);

	pthread_spin_lock(&data->rx_lock);

	net_pkt_read(pkt, data->rx_buffer, len);
	data->res.length = len;
	data->has_pkt = true;

	pthread_spin_unlock(&data->rx_lock);

	k_work_submit(&data->rx_work); /* TODO: use polling for now */
	/* TODO: sw-240 (Tx) */
	return 0;
}

static const struct ethernet_api eth_tsn_nic_api = {
	.iface_api.init = eth_tsn_nic_iface_init,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_tsn_nic_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
	.start = eth_tsn_nic_start,
	.stop = eth_tsn_nic_stop,
	.get_capabilities = eth_tsn_nic_get_capabilities,
	.set_config = eth_tsn_nic_set_config,
	.get_config = eth_tsn_nic_get_config,
#if defined(CONFIG_NET_VLAN)
	.vlan_setup = eth_tsn_nic_vlan_setup,
#endif /* CONFIG_NET_VLAN */
#if defined(CONFIG_PTP_CLOCK)
	.get_ptp_clock = eth_tsn_nic_get_ptp_clock,
#endif /* CONFIG_PTP_CLOCK */
	.get_phy = eth_tsn_nic_get_phy,
	.send = eth_tsn_nic_send,
};

static int eth_tsn_nic_init(const struct device *dev)
{
	const struct eth_tsn_nic_config *config = dev->config;
	struct eth_tsn_nic_data *data = dev->data;

	pthread_spin_init(&data->tx_lock, PTHREAD_PROCESS_PRIVATE);
	pthread_spin_init(&data->rx_lock, PTHREAD_PROCESS_PRIVATE);

	/* TODO: Select proper values for the first three bytes */
	gen_random_mac(data->mac_addr, 0x0, 0x0, 0xab);

	k_work_init(&data->rx_work, eth_tsn_nic_rx);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), eth_tsn_nic_isr,
		    DEVICE_DT_INST_GET(0), 0);

	/* Test logs */
	mm_reg_t test_dma_addr;

	device_map(&test_dma_addr, 0x1b08000000, 0x4000, K_MEM_CACHE_NONE);
	printk("H2C engine id: 0x%08x\n", sys_read32(test_dma_addr));
	printk("H2C engine control: 0x%08x\n", sys_read32(test_dma_addr + 0x0004));
	dma_start(config->dma_dev, 0);
	printk("H2C engine start\n");
	printk("H2C engine control: 0x%08x\n", sys_read32(test_dma_addr + 0x0004));
	printk("C2H engine control: 0x%08x\n", sys_read32(test_dma_addr + 0x1004));

	return 0;
}

/* TODO: priority should be CONFIG_ETH_INIT_PRIORITY */
#define ETH_TSN_NIC_INIT(n)                                                                        \
	static struct eth_tsn_nic_data eth_tsn_nic_data_##n = {};                                  \
                                                                                                   \
	static const struct eth_tsn_nic_config eth_tsn_nic_cfg_##n = {                             \
		.dma_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(n))),                               \
		.pci_dev = DEVICE_DT_GET(DT_GPARENT(DT_DRV_INST(n))),                              \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_tsn_nic_init, NULL, &eth_tsn_nic_data_##n,            \
				      &eth_tsn_nic_cfg_##n, 99, &eth_tsn_nic_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_TSN_NIC_INIT)
