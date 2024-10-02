/*
 * Copyright (c) 2024 Junho Lee <junho@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* TODO: Better name */
#define DT_DRV_COMPAT xilinx_alinx_dma

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(xilinx_dma, LOG_LEVEL_ERR);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/controller.h>

#define XDMA_ID_H2C 0x1fc0
#define XDMA_ID_C2H 0x1fc1

#define XDMA_CHANNEL_ID_MASK 0x00000f00
#define XDMA_CHANNEL_ID_LSB 8
#define XDMA_ENGINE_ID_MASK 0xffff0000
#define XDMA_ENGINE_ID_LSB 16

#define XDMA_ALIGN_BYTES_MASK 0x00ff0000
#define XDMA_ALIGN_BYTES_LSB 16
#define XDMA_GRANULARITY_BYTES_MASK 0x0000ff00
#define XDMA_GRANULARITY_BYTES_LSB 8
#define XDMA_ADDRESS_BITS_MASK 0x000000ff
#define XDMA_ADDRESS_BITS_LSB 0

#define XDMA_CTRL_IE_DESC_STOPPED (1UL << 1)
#define XDMA_CTRL_IE_DESC_COMPLETED (1UL << 2)
#define XDMA_CTRL_IE_DESC_ALIGN_MISMATCH (1UL << 3)
#define XDMA_CTRL_IE_MAGIC_STOPPED (1UL << 4)
#define XDMA_CTRL_IE_IDLE_STOPPED (1UL << 6)
#define XDMA_CTRL_IE_READ_ERROR (1UL << 9)
#define XDMA_CTRL_IE_DESC_ERROR (1UL << 19)

#define XDMA_CTRL_NON_INCR_ADDR (1UL << 25)

#define XDMA_H2C 0
#define XDMA_C2H 1

#define XDMA_C2H_OFFSET 0x1000

#define XDMA_CONFIG_BAR_NUM 1
/* Size of BAR1, it needs to be hard-coded as there is no PCIe API for this */
#define XDMA_CONFIG_BAR_SIZE 0x10000

#define XDMA_ENGINE_START 16268831
#define XDMA_ENGINE_STOP 16268830

struct dma_xilinx_config_regs {
	uint32_t identifier;
	uint32_t reserved_1[4];
	uint32_t msi_enable;
};

struct dma_xilinx_engine_regs {
	uint32_t identifier;
	uint32_t control;
	uint32_t control_w1s;
	uint32_t control_w1c;
	uint32_t reserved_1[12];	/* padding */

	uint32_t status;
	uint32_t status_rc;
	uint32_t completed_desc_count;
	uint32_t alignments;
	uint32_t reserved_2[14];	/* padding */

	uint32_t poll_mode_wb_lo;
	uint32_t poll_mode_wb_hi;
	uint32_t interrupt_enable_mask;
	uint32_t interrupt_enable_mask_w1s;
	uint32_t interrupt_enable_mask_w1c;
	uint32_t reserved_3[9];	/* padding */

	uint32_t perf_ctrl;
	uint32_t perf_cyc_lo;
	uint32_t perf_cyc_hi;
	uint32_t perf_dat_lo;
	uint32_t perf_dat_hi;
	uint32_t perf_pnd_lo;
	uint32_t perf_pnd_hi;
} __packed;  // TODO: Move these to header file

struct dma_xilinx_config {
	const struct device *pci_dev;
};

struct dma_xilinx_data {
	mm_reg_t bar;
	struct dma_xilinx_engine_regs *regs[2];
};

static int dma_xilinx_config(const struct device *dev, uint32_t channel, struct dma_config *config) {
	return -ENOTSUP;
}

static int dma_xilinx_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst, size_t size) {
	return -ENOTSUP;
}

static int dma_xilinx_start(const struct device *dev, uint32_t channel) {
    const struct dma_xilinx_data * data = dev->data;

	/* There is only one channel for each direction for now */
	sys_write32(XDMA_ENGINE_START, &data->regs[XDMA_H2C]->control);
	sys_write32(XDMA_ENGINE_START, &data->regs[XDMA_C2H]->control);

    return 0;
}

static int dma_xilinx_stop(const struct device *dev, uint32_t channel) {
    const struct dma_xilinx_data * data = dev->data;

	/* There is only one channel for each direction for now */
	sys_write32(XDMA_ENGINE_STOP, &data->regs[XDMA_H2C]->control);
	sys_write32(XDMA_ENGINE_STOP, &data->regs[XDMA_C2H]->control);

    return 0;
}

static int dma_xilinx_suspend(const struct device *dev, uint32_t channel) {
	return -ENOTSUP;
}

static int dma_xilinx_resume(const struct device *dev, uint32_t channel) {
	return -ENOTSUP;
}

static int dma_xilinx_get_status(const struct device *dev, uint32_t channel, struct dma_status *status) {
	return -ENOTSUP;
}

static int dma_xilinx_get_attribute(const struct device *dev, uint32_t type, uint32_t *value) {
	return -ENOTSUP;
}

static bool dma_xilinx_chan_filter(const struct device *dev, int channel, void *filter_param) {
	return -ENOTSUP;
}

static const struct dma_driver_api dma_xilinx_api = {
	.config = dma_xilinx_config,
	.reload = dma_xilinx_reload,
	.start = dma_xilinx_start,
	.stop = dma_xilinx_stop,
	.suspend = dma_xilinx_suspend,
	.resume = dma_xilinx_resume,
	.get_status = dma_xilinx_get_status,
	.get_attribute = dma_xilinx_get_attribute,
	.chan_filter = dma_xilinx_chan_filter,
};

static int get_engine_channel_id(struct dma_xilinx_engine_regs *regs) {
	int value = sys_read32(&regs->identifier);
	return (value & XDMA_CHANNEL_ID_MASK) >> XDMA_CHANNEL_ID_LSB;
}

static int get_engine_id(struct dma_xilinx_engine_regs *regs) {
	int value = sys_read32(&regs->identifier);
	return (value & XDMA_ENGINE_ID_MASK) >> XDMA_ENGINE_ID_LSB;
}

static int engine_init_regs(struct dma_xilinx_engine_regs *regs) {
	uint32_t align_bytes, granularity_bytes, address_bits;
	uint32_t tmp;

	sys_write32(XDMA_CTRL_NON_INCR_ADDR, &regs->control_w1c);
	tmp = sys_read32(&regs->alignments);
	if (tmp != 0) {
		align_bytes = (tmp & XDMA_ALIGN_BYTES_MASK) >> XDMA_ALIGN_BYTES_LSB;
		granularity_bytes = (tmp & XDMA_GRANULARITY_BYTES_MASK) >> XDMA_GRANULARITY_BYTES_LSB;
		address_bits = (tmp & XDMA_ADDRESS_BITS_MASK) >> XDMA_ADDRESS_BITS_LSB;
	} else {
		align_bytes = 1;
		granularity_bytes = 1;
		address_bits = 64;
	}

	tmp = XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	tmp |= XDMA_CTRL_IE_MAGIC_STOPPED;
	tmp |= XDMA_CTRL_IE_IDLE_STOPPED;
	tmp |= XDMA_CTRL_IE_READ_ERROR;
	tmp |= XDMA_CTRL_IE_DESC_ERROR;
	tmp |= XDMA_CTRL_IE_DESC_STOPPED;
	tmp |= XDMA_CTRL_IE_DESC_COMPLETED;

	sys_write32(tmp, &regs->interrupt_enable_mask);

	return 0;
}

static int dma_xilinx_init(const struct device *dev) {
    const struct dma_xilinx_config * config = dev->config;
    struct dma_xilinx_data * data = dev->data;
	struct dma_xilinx_engine_regs * regs;
	uintptr_t bar_addr, bus_addr;
	int engine_id, channel_id;
	bool ret;

	// request_regions()
	ret = pcie_ctrl_region_allocate(config->pci_dev, PCIE_BDF(XDMA_CONFIG_BAR_NUM, 0, 0), true, false, XDMA_CONFIG_BAR_SIZE, &bus_addr);
	if (!ret) {
		return -EINVAL;
	}

	ret = pcie_ctrl_region_translate(config->pci_dev, PCIE_BDF(XDMA_CONFIG_BAR_NUM, 0, 0), true, false, bus_addr, &bar_addr);
	if (!ret) {
		return -EINVAL;
	}

	// map_bars()
	device_map(&data->bar, bar_addr, XDMA_CONFIG_BAR_SIZE, K_MEM_CACHE_NONE);

	// probe_engines()
	// H2C: register 0x0000
	regs = data->bar;
	engine_id = get_engine_id(regs);
	channel_id = get_engine_channel_id(regs);
	if ((engine_id != XDMA_ID_H2C) || (channel_id != 0)) {
		return -EINVAL;
	}

	// engine_init()
	engine_init_regs(regs);
	data->regs[XDMA_H2C] = regs;

	// C2H: register 0x1000
	regs = data->bar + XDMA_C2H_OFFSET;
	engine_id = get_engine_id(regs);
	channel_id = get_engine_channel_id(regs);
	if ((engine_id != XDMA_ID_C2H) || (channel_id != 0)) {
		return -EINVAL;
	}
	data->regs[XDMA_C2H] = regs;

	return 0;
}


/* TODO: POST_KERNEL is set to use printk, revert this after the development is done */
#define DMA_XILINX_INIT(n)                                                                       \
	static struct dma_xilinx_data dma_xilinx_data_##n = {};                                \
                                                                                                   \
	static const struct dma_xilinx_config dma_xilinx_cfg_##n = {                                 \
		.pci_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(n))), \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dma_xilinx_init, NULL, &dma_xilinx_data_##n,                  \
			      &dma_xilinx_cfg_##n, POST_KERNEL, 99, &dma_xilinx_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XILINX_INIT)