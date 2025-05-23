/*
 * Copyright (c) 2024 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* TODO: Better name */
#define DT_DRV_COMPAT tsnlab_tsn_nic_eth

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_tsn_nic, LOG_LEVEL_DBG);

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/controller.h>

#include <zephyr/posix/posix_types.h>
#include <zephyr/posix/pthread.h>
#include <zephyr/posix/time.h>

#include "eth.h"
#include "eth_tsn_nic_priv.h"

#define RX_DESC_NODE DT_NODELABEL(tsn_dma0)
#define RES_NODE     DT_NODELABEL(tsn_res_region)

#define RX_DESC_ADDR DT_REG_ADDR(RX_DESC_NODE)
#define RES_ADDR     DT_REG_ADDR(RES_NODE)

#define TSN_REG_VERSION         0x0000
#define TSN_REG_CONFIG          0x0004
#define TSN_REG_CONTROL         0x0008
#define TSN_REG_SCRATCH         0x0010
#define TSN_REG_QBV_SLOT_STATUS 0x0028
#define TSN_REG_PULSE_AT_HI     0x002C
#define TSN_REG_PULSE_AT_LO     0x0030
#define TSN_REG_CYCLE_1S        0x0034

#define TSN_REG_TEMAC_STATUS 0x0500

static inline uint32_t fpga_reg_read(const struct device *dev, uint32_t offset)
{
	struct eth_tsn_nic_data *data = dev->data;

	return sys_read32((mem_addr_t)(data->bar[0] + offset));
}

void tsn_print_top_registers(const struct device *dev)
{
	LOG_DBG(">>> TSN TOP Register Group <<<");
	LOG_DBG("TSN_VERSION          (0x%04x): 0x%08x", TSN_REG_VERSION,
		fpga_reg_read(dev, TSN_REG_VERSION));
	LOG_DBG("TSN_CONFIG           (0x%04x): 0x%08x", TSN_REG_CONFIG,
		fpga_reg_read(dev, TSN_REG_CONFIG));
	LOG_DBG("TSN_CONTROL          (0x%04x): 0x%08x", TSN_REG_CONTROL,
		fpga_reg_read(dev, TSN_REG_CONTROL));
	LOG_DBG("SCRATCH              (0x%04x): 0x%08x", TSN_REG_SCRATCH,
		fpga_reg_read(dev, TSN_REG_SCRATCH));
	LOG_DBG("QBV_SLOT_STATUS      (0x%04x): 0x%08x", TSN_REG_QBV_SLOT_STATUS,
		fpga_reg_read(dev, TSN_REG_QBV_SLOT_STATUS));
	LOG_DBG("PULSE_AT_HI [63:32]  (0x%04x): 0x%08x", TSN_REG_PULSE_AT_HI,
		fpga_reg_read(dev, TSN_REG_PULSE_AT_HI));
	LOG_DBG("PULSE_AT_LO [31:0]   (0x%04x): 0x%08x", TSN_REG_PULSE_AT_LO,
		fpga_reg_read(dev, TSN_REG_PULSE_AT_LO));
	LOG_DBG("CYCLE_1S             (0x%04x): 0x%08x", TSN_REG_CYCLE_1S,
		fpga_reg_read(dev, TSN_REG_CYCLE_1S));
	LOG_DBG("TEMAC_STATUS         (0x%04x): 0x%08x", TSN_REG_TEMAC_STATUS,
		fpga_reg_read(dev, TSN_REG_TEMAC_STATUS));
}

void dump_dma_h2c_all_regs(struct dma_tsn_nic_engine_regs *regs)
{
	LOG_DBG("=== DMA Engine Registers ===");

	LOG_DBG("identifier              : 0x%08x", regs->identifier);
	LOG_DBG("control                 : 0x%08x", regs->control);
	LOG_DBG("control_w1s             : 0x%08x", regs->control_w1s);
	LOG_DBG("control_w1c             : 0x%08x", regs->control_w1c);

	LOG_DBG("status                  : 0x%08x", regs->status);
	LOG_DBG("status_rc               : 0x%08x", regs->status_rc);
	LOG_DBG("completed_desc_count    : 0x%08x", regs->completed_desc_count);
	LOG_DBG("alignments              : 0x%08x", regs->alignments);

	LOG_DBG("poll_mode_wb_lo         : 0x%08x", regs->poll_mode_wb_lo);
	LOG_DBG("poll_mode_wb_hi         : 0x%08x", regs->poll_mode_wb_hi);

	LOG_DBG("interrupt_enable_mask   : 0x%08x", regs->interrupt_enable_mask);
	LOG_DBG("int_en_mask_w1s         : 0x%08x", regs->interrupt_enable_mask_w1s);
	LOG_DBG("int_en_mask_w1c         : 0x%08x", regs->interrupt_enable_mask_w1c);

	LOG_DBG("perf_ctrl               : 0x%08x", regs->perf_ctrl);
	LOG_DBG("perf_cyc_lo             : 0x%08x", regs->perf_cyc_lo);
	LOG_DBG("perf_cyc_hi             : 0x%08x", regs->perf_cyc_hi);
	LOG_DBG("perf_dat_lo             : 0x%08x", regs->perf_dat_lo);
	LOG_DBG("perf_dat_hi             : 0x%08x", regs->perf_dat_hi);
	LOG_DBG("perf_pnd_lo             : 0x%08x", regs->perf_pnd_lo);
	LOG_DBG("perf_pnd_hi             : 0x%08x", regs->perf_pnd_hi);

	LOG_DBG("===========================");
}

static void eth_tsn_check_status(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pcie1), okay)
	LOG_DBG("PCIe controller is okay.");
#else
	LOG_ERR("PCIe controller is NOT okay.");
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(eth), okay)
	const struct device *eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth));

	if (!device_is_ready(eth_dev)) {
		LOG_ERR("Ethernet device %s is NOT ready.", eth_dev->name);
	} else {
		LOG_DBG("Ethernet device %s is ready.", eth_dev->name);
	}
#else
	LOG_ERR("Ethernet device node is not okay in Device Tree.");
#endif
}

static void eth_tsn_nic_isr(const struct device *dev)
{
	/* TODO: Implement interrupts */
	ARG_UNUSED(dev);
}

static void tx_desc_set(struct dma_tsn_nic_desc *desc, uintptr_t addr, uint32_t len)
{
	uint32_t control;

	desc->control = sys_cpu_to_le32(DESC_MAGIC);
	control = sys_le32_to_cpu(desc->control & ~(LS_BYTE_MASK));
	control |= DESC_STOPPED;
	control |= DESC_EOP;
	control |= DESC_COMPLETED;
	desc->control = sys_cpu_to_le32(control);

	desc->dst_addr_lo = 0;
	desc->dst_addr_hi = 0;

	desc->src_addr_lo = sys_cpu_to_le32(PCI_DMA_L(addr));
	desc->src_addr_hi = sys_cpu_to_le32(PCI_DMA_H(addr));
	desc->bytes = sys_cpu_to_le32(len);
}

static void rx_desc_set(struct dma_tsn_nic_desc *desc, uintptr_t addr, uint32_t len)
{
	uint32_t control;

	desc->control = sys_cpu_to_le32(DESC_MAGIC);
	control = sys_le32_to_cpu(desc->control & ~(LS_BYTE_MASK));
	control |= DESC_STOPPED;
	control |= DESC_EOP;
	control |= DESC_COMPLETED;
	desc->control = sys_cpu_to_le32(control);

	desc->src_addr_lo = 0;
	desc->src_addr_hi = 0;

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
	int ret = 0;

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
		LOG_ERR("alloc failed\n");
		goto done;
	}

	ret = net_pkt_write(pkt, data->rx_buffer.data, pkt_len);
	if (ret != 0) {
		LOG_ERR("write failed\n");
		goto done;
	}

	ret = net_recv_data(data->iface, pkt);
	if (ret != 0) {
		LOG_ERR("recv failed\n");
		goto done;
	}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	/* Not sure this is how it's supposed to be used as there are not much examples */
	if (net_pkt_is_rx_timestamping(pkt)) {
		/* FIXME: Get HW timestamp */
		timestamp.second = UINT64_MAX;
		timestamp.nanosecond = 999999999; /* 1s - 1ns */
		net_pkt_set_timestamp(pkt, &timestamp);
	}
#else
	ARG_UNUSED(timestamp);
#endif /* CONFIG_NET_PKT_TIMESTMP */

done:

	if (ret != 0 && pkt != NULL) { /* Handle errors */
		net_pkt_unref(pkt);
	}

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

	net_if_set_link_addr(iface, data->mac_addr, ETH_ALEN, NET_LINK_ETHERNET);
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

	data->rx_desc.src_addr_lo = sys_cpu_to_le32(PCI_DMA_L((uintptr_t)&data->res));
	data->rx_desc.src_addr_hi = sys_cpu_to_le32(PCI_DMA_H((uintptr_t)&data->res));

	rx_desc_set(&data->rx_desc, (uintptr_t)&data->rx_buffer, BUFFER_SIZE);

	/**
	 * TODO: Find out how to move this to dma driver
	 * or how to access dma registers from here
	 */

	pthread_spin_lock(&data->rx_lock);

	/* FIXME: It seems the board is not reading the descriptor properly */
	sys_read32((uintptr_t)&data->regs[DMA_H2C]->status_rc); /* Clear status regs */
	sys_write32(sys_cpu_to_le32(PCI_DMA_L((uintptr_t)&data->rx_desc)),
		    (uintptr_t)&data->sgdma_regs[DMA_H2C]->first_desc_lo);
	sys_write32(sys_cpu_to_le32(PCI_DMA_H((uintptr_t)&data->rx_desc)),
		    (uintptr_t)&data->sgdma_regs[DMA_H2C]->first_desc_hi);
	sys_write32(DMA_ENGINE_START, (uintptr_t)&data->regs[DMA_H2C]->control);

	pthread_spin_unlock(&data->rx_lock);

	return 0;
}

static int eth_tsn_nic_stop(const struct device *dev)
{
	/* TODO: sw-257 (Misc. APIs) */
	struct eth_tsn_nic_data *data = dev->data;
	sys_write32(DMA_ENGINE_STOP, (mem_addr_t)&data->regs[DMA_H2C]->control);
	sys_write32(DMA_ENGINE_STOP, (mem_addr_t)&data->regs[DMA_C2H]->control);
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
	int ret;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_QBV_PARAM:
		ret = tsn_set_qbv(dev, config->qbv_param);
		break;
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		ret = tsn_set_qav(dev, config->qav_param);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
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
	struct timespec ts;
	uint64_t now;
	uint32_t w;
	size_t len;
	int ret;

#if 0 /* Rx test */
	len = net_pkt_get_len(pkt);

	pthread_spin_lock(&data->rx_lock);

	net_pkt_read(pkt, data->rx_buffer.data, len);
	data->res.length = len;
	data->has_pkt = true;

	pthread_spin_unlock(&data->rx_lock);

	k_work_submit(&data->rx_work); /* TODO: use polling for now */
#endif

	tsn_print_top_registers(dev);

	eth_tsn_check_status();

	pthread_spin_lock(&data->tx_lock);

	len = net_pkt_get_len(pkt);

	memset(data->tx_buffer.data, 0, sizeof(data->tx_buffer.data));

	ret = net_pkt_read(pkt, data->tx_buffer.data, len);
	if (ret != 0) {
		LOG_ERR("data length: %lu (net_pkt_read: %d)\n", len, ret);
		goto error;
	}

	if (len < ETH_ZLEN) {
		len = ETH_ZLEN;
	}

	data->tx_buffer.metadata.frame_length = len;

	LOG_DBG("data length: %d\n", data->tx_buffer.metadata.frame_length);

	ret = clock_gettime(CLOCK_MONOTONIC, &ts); /* TODO: Replace with HW clock */
	if (ret != 0) {
		LOG_ERR("clock_gettime error: %d\n", ret);
		goto error;
	}

	/* This will not work after July, 2554 because of overflow */
	now = ts.tv_sec * NS_IN_1S + ts.tv_nsec;
	ret = tsn_fill_metadata(dev, now, &data->tx_buffer);
	if (ret != 0) {
		goto error;
	}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	/* TODO: Timestamps */
#endif /* CONFIG_NET_PKT_TIMESTMP */

	tx_desc_set(&data->tx_desc, (uintptr_t)&data->tx_buffer, len + TX_METADATA_SIZE);

	w = sys_cpu_to_le32(PCI_DMA_L((uintptr_t)&data->tx_desc));
	sys_write32(w, data->bar[DMA_CONFIG_BAR_IDX] + DESC_REG_LO);
	w = sys_cpu_to_le32(PCI_DMA_H((uintptr_t)&data->tx_desc));
	sys_write32(w, data->bar[DMA_CONFIG_BAR_IDX] + DESC_REG_HI);
	sys_write32(0, data->bar[DMA_CONFIG_BAR_IDX] + DESC_REG_HI);
	sys_write32(0, data->bar[DMA_CONFIG_BAR_IDX] + DESC_REG_HI + 4);
	sys_write32(DMA_ENGINE_START, (uintptr_t)&data->regs[DMA_H2C]->control);

	LOG_INF("control: %08x", data->regs[DMA_H2C]->control);
	LOG_INF("status: %08x", data->regs[DMA_H2C]->status);
	LOG_INF("status_rc: %08x", data->regs[DMA_H2C]->status_rc);

	LOG_DESC(&data->tx_desc);

	dump_dma_h2c_all_regs(data->regs[DMA_H2C]);

	/* offset = completed_desc_count */
	uint32_t completed =
		sys_read32((uintptr_t)data->regs[DMA_H2C] + 0x48);

	LOG_DBG("Completed Desc Count: %d\n", completed);

	uint32_t nCompleted = 0;

	for (int i = 0; i < 100000; ++i) {
		uint32_t completed = sys_read32((uintptr_t)data->regs[DMA_H2C] + 0x48);

		if (completed > 0) {
			LOG_DBG("DMA completed!\n");
			nCompleted++;
			break;
		}
		k_busy_wait(10); /* 10 µs delay */
	}

	/* TODO: This should be done in tsn_nic_isr() */
	pthread_spin_unlock(&data->tx_lock);

	return 0;

error:
	pthread_spin_unlock(&data->tx_lock);
	return ret;
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

static int get_engine_channel_id(struct dma_tsn_nic_engine_regs *regs)
{
	int value = sys_read32((mem_addr_t)&regs->identifier);

	return (value & DMA_CHANNEL_ID_MASK) >> DMA_CHANNEL_ID_LSB;
}

static int get_engine_id(struct dma_tsn_nic_engine_regs *regs)
{
	int value = sys_read32((mem_addr_t)&regs->identifier);

	return (value & DMA_ENGINE_ID_MASK) >> DMA_ENGINE_ID_LSB;
}

static int engine_init_regs(struct dma_tsn_nic_engine_regs *regs)
{
	uint32_t align_bytes, granularity_bytes, address_bits;
	uint32_t tmp, flags;

	sys_write32(DMA_CTRL_NON_INCR_ADDR, (mem_addr_t)&regs->control_w1c);
	tmp = sys_read32((mem_addr_t)&regs->alignments);
	/* These values will be used in other operations */
	if (tmp != 0) {
		align_bytes = (tmp & DMA_ALIGN_BYTES_MASK) >> DMA_ALIGN_BYTES_LSB;
		granularity_bytes = (tmp & DMA_GRANULARITY_BYTES_MASK) >> DMA_GRANULARITY_BYTES_LSB;
		address_bits = (tmp & DMA_ADDRESS_BITS_MASK) >> DMA_ADDRESS_BITS_LSB;
	} else {
		align_bytes = 1;
		granularity_bytes = 1;
		address_bits = 64;
	}

	flags = (DMA_CTRL_IE_DESC_ALIGN_MISMATCH | DMA_CTRL_IE_MAGIC_STOPPED |
		 DMA_CTRL_IE_IDLE_STOPPED | DMA_CTRL_IE_READ_ERROR | DMA_CTRL_IE_DESC_ERROR |
		 DMA_CTRL_IE_DESC_STOPPED | DMA_CTRL_IE_DESC_COMPLETED);

	sys_write32(flags, (mem_addr_t)&regs->interrupt_enable_mask);

	flags = (DMA_CTRL_RUN_STOP | DMA_CTRL_IE_READ_ERROR | DMA_CTRL_IE_DESC_ERROR |
		 DMA_CTRL_IE_DESC_ALIGN_MISMATCH | DMA_CTRL_IE_MAGIC_STOPPED |
		 DMA_CTRL_POLL_MODE_WB);

	sys_write32(flags, (mem_addr_t)&regs->control);

	return 0;
}

static int map_bar(const struct device *dev, int idx, size_t size)
{
	const struct eth_tsn_nic_config *config = dev->config;
	struct eth_tsn_nic_data *data = dev->data;
	uintptr_t bar_addr, bus_addr;
	bool ret;

	ret = pcie_ctrl_region_allocate(config->pci_dev, PCIE_BDF(idx, 0, 0), true, false,
					DMA_CONFIG_BAR_SIZE, &bus_addr);
	if (!ret) {
		return -EINVAL;
	}

	ret = pcie_ctrl_region_translate(config->pci_dev, PCIE_BDF(idx, 0, 0), true, false,
					 bus_addr, &bar_addr);
	if (!ret) {
		return -EINVAL;
	}

	device_map(&data->bar[idx], bar_addr, size, K_MEM_CACHE_NONE);

	return 0;
}

static int eth_tsn_nic_init(const struct device *dev)
{
	struct eth_tsn_nic_data *data = dev->data;
	struct dma_tsn_nic_engine_regs *regs;
	int engine_id, channel_id;

	LOG_DBG("device name: %s", dev->name);

	if (map_bar(dev, 0, 0x1000) != 0) {
		LOG_ERR("Failed to map BAR 0\n");
		return -EINVAL;
	}

	if (map_bar(dev, DMA_CONFIG_BAR_IDX, DMA_CONFIG_BAR_SIZE) != 0) {
		return -EINVAL;
	}

	regs = (struct dma_tsn_nic_engine_regs *)(data->bar[DMA_CONFIG_BAR_IDX]);
	engine_id = get_engine_id(regs);
	channel_id = get_engine_channel_id(regs);
	if ((engine_id != DMA_ID_H2C) || (channel_id != 0)) {
		return -EINVAL;
	}

	engine_init_regs(regs);
	data->regs[DMA_H2C] = regs;
	data->sgdma_regs[DMA_H2C] =
		(struct dma_tsn_nic_engine_sgdma_regs *)(data->bar[DMA_CONFIG_BAR_IDX] +
							 SGDMA_OFFSET_FROM_CHANNEL);

	regs = (struct dma_tsn_nic_engine_regs *)(data->bar[DMA_CONFIG_BAR_IDX] + DMA_C2H_OFFSET);
	engine_id = get_engine_id(regs);
	channel_id = get_engine_channel_id(regs);
	if ((engine_id != DMA_ID_C2H) || (channel_id != 0)) {
		return -EINVAL;
	}

	engine_init_regs(regs);
	data->regs[DMA_C2H] = regs;
	data->sgdma_regs[DMA_C2H] =
		(struct dma_tsn_nic_engine_sgdma_regs *)(data->bar[DMA_CONFIG_BAR_IDX] +
							 SGDMA_OFFSET_FROM_CHANNEL +
							 DMA_C2H_OFFSET);

	/* TSN registers */
	sys_write32(0x1, data->bar[0] + 0x0008);
	sys_write32(0x800f0000, data->bar[0] + 0x0610);
	sys_write32(0x10, data->bar[0] + 0x0620);

	tsn_print_top_registers(dev);

	pthread_spin_init(&data->tx_lock, PTHREAD_PROCESS_PRIVATE);
	pthread_spin_init(&data->rx_lock, PTHREAD_PROCESS_PRIVATE);

	/* TODO: Select proper values for the first three bytes */
	gen_random_mac(data->mac_addr, 0x0, 0x0, 0xab);

	k_work_init(&data->rx_work, eth_tsn_nic_rx);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), eth_tsn_nic_isr,
		    DEVICE_DT_INST_GET(0), 0);

	/* Test logs */
	printk("H2C engine id: 0x%08x\n", sys_read32((mem_addr_t)&data->regs[DMA_H2C]->identifier));
	printk("H2C engine control: 0x%08x\n",
	       sys_read32((mem_addr_t)&data->regs[DMA_H2C]->control));
	printk("C2H engine id: 0x%08x\n", sys_read32((mem_addr_t)&data->regs[DMA_C2H]->identifier));
	printk("C2H engine control: 0x%08x\n",
	       sys_read32((mem_addr_t)&data->regs[DMA_C2H]->control));

	printk("MAC addr: %02X-%02X-%02X-%02X-%02X-%02X\n", data->mac_addr[0], data->mac_addr[1],
	       data->mac_addr[2], data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
	return 0;
}

/* TODO: priority should be CONFIG_ETH_INIT_PRIORITY */
#define ETH_TSN_NIC_INIT(n)                                                                        \
	static struct eth_tsn_nic_data eth_tsn_nic_data_##n = {};                                  \
                                                                                                   \
	static const struct eth_tsn_nic_config eth_tsn_nic_cfg_##n = {                             \
		.pci_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(n))),                               \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_tsn_nic_init, NULL, &eth_tsn_nic_data_##n,            \
				      &eth_tsn_nic_cfg_##n, 99, &eth_tsn_nic_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_TSN_NIC_INIT)
