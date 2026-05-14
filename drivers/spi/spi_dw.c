/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2023 Synopsys, Inc. All rights reserved.
 * Copyright (c) 2023 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_spi

/* spi_dw.c - Designware SPI driver implementation */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_dw);

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_IOAPIC
#include <zephyr/drivers/interrupt_controller/ioapic.h>
#endif

#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>

#include "spi_dw.h"
#include "spi_context.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

static inline bool spi_dw_is_slave(struct spi_dw_data *spi)
{
	return (IS_ENABLED(CONFIG_SPI_SLAVE) &&
		spi_context_is_slave(&spi->ctx));
}

static void completed(const struct device *dev, int error)
{
	struct spi_dw_data *spi = dev->data;
	struct spi_context *ctx = &spi->ctx;

	if (error) {
		goto out;
	}

	if (spi_context_tx_on(&spi->ctx) ||
	    spi_context_rx_on(&spi->ctx)) {
		return;
	}

out:
	/* need to give time for FIFOs to drain before issuing more commands */
	while (test_bit_sr_busy(dev)) {
	}

	/* Disabling interrupts */
	write_imr(dev, DW_SPI_IMR_MASK);
	/* Disabling the controller */
	clear_bit_ssienr(dev);

	if (!spi_dw_is_slave(spi)) {
		if (spi_cs_is_gpio(ctx->config)) {
			spi_context_cs_control(ctx, false);
		} else {
			write_ser(dev, 0);
		}
	}

	LOG_DBG("SPI transaction completed %s error",
		    error ? "with" : "without");

	spi_context_complete(&spi->ctx, dev, error);
}

static void push_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t data = 0U;
	uint32_t f_tx;

	if (spi_context_rx_on(&spi->ctx)) {
		f_tx = info->fifo_depth - read_txflr(dev) -
			read_rxflr(dev);
		if ((int)f_tx < 0) {
			f_tx = 0U; /* if rx-fifo is full, hold off tx */
		}
	} else {
		f_tx = info->fifo_depth - read_txflr(dev);
	}

	while (f_tx) {
		if (spi_context_tx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				data = UNALIGNED_GET((uint8_t *)
						     (spi->ctx.tx_buf));
				break;
			case 2:
				data = UNALIGNED_GET((uint16_t *)
						     (spi->ctx.tx_buf));
				break;
			case 4:
				data = UNALIGNED_GET((uint32_t *)
						     (spi->ctx.tx_buf));
				break;
			}
		} else if (spi_context_rx_on(&spi->ctx)) {
			/* No need to push more than necessary */
			if ((int)(spi->ctx.rx_len - spi->fifo_diff) <= 0) {
				break;
			}

			data = 0U;
		} else if (spi_context_tx_on(&spi->ctx)) {
			data = 0U;
		} else {
			/* Nothing to push anymore */
			break;
		}

		write_dr(dev, data);

		spi_context_update_tx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff++;

		f_tx--;
	}

	if (!spi_context_tx_on(&spi->ctx)) {
		/* prevents any further interrupts demanding TX fifo fill */
		write_txftlr(dev, 0);
	}
}

static void pull_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	while (read_rxflr(dev)) {
		uint32_t data = read_dr(dev);

		if (spi_context_rx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				UNALIGNED_PUT(data, (uint8_t *)spi->ctx.rx_buf);
				break;
			case 2:
				UNALIGNED_PUT(data, (uint16_t *)spi->ctx.rx_buf);
				break;
			case 4:
				UNALIGNED_PUT(data, (uint32_t *)spi->ctx.rx_buf);
				break;
			}
		}

		if (spi_context_rx_on(&spi->ctx) && (spi->fifo_diff > 0U)) {
			spi_context_update_rx(&spi->ctx, spi->dfs, 1);
			spi->fifo_diff--;
		}
	}

	if (!spi->ctx.rx_len) {
		if (spi->ctx.tx_len && (spi->ctx.tx_len < info->fifo_depth)) {
			write_rxftlr(dev, spi->ctx.tx_len - 1);
		}
	} else if (read_rxftlr(dev) >= spi->ctx.rx_len) {
		write_rxftlr(dev, spi->ctx.rx_len - 1);
	}
}

static int spi_dw_configure(const struct device *dev,
			    struct spi_dw_data *spi,
			    const struct spi_config *config)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t ctrlr0 = 0U;

	LOG_DBG("%p (prev %p)", config, spi->ctx.config);

	if (spi_context_configured(&spi->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Verify if requested op mode is relevant to this controller */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		if (!(info->serial_target)) {
			LOG_ERR("Slave mode not supported");
			return -ENOTSUP;
		}
	} else {
		if (info->serial_target) {
			LOG_ERR("Master mode not supported");
			return -ENOTSUP;
		}
	}

	if ((config->operation & SPI_TRANSFER_LSB) ||
	    (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	     (config->operation & (SPI_LINES_DUAL |
				   SPI_LINES_QUAD | SPI_LINES_OCTAL)))) {
		LOG_ERR("Unsupported configuration");
		return -EINVAL;
	}

	if (info->max_xfer_size < SPI_WORD_SIZE_GET(config->operation)) {
		LOG_ERR("Max xfer size is %u, word size of %u not allowed",
			info->max_xfer_size, SPI_WORD_SIZE_GET(config->operation));
		return -ENOTSUP;
	}

	/* Word size */
	if (!IS_ENABLED(CONFIG_SPI_DW_HSSI) && (info->max_xfer_size == 32)) {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_32(SPI_WORD_SIZE_GET(config->operation));
	} else {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_16(SPI_WORD_SIZE_GET(config->operation));
	}

	/* Determine how many bytes are required per-frame */
	spi->dfs = SPI_WS_TO_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPOL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPH;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		ctrlr0 |= DW_SPI_CTRLR0_SRL;
	}

	/* Installing the configuration */
	write_ctrlr0(dev, ctrlr0);

	/* At this point, it's mandatory to set this on the context! */
	spi->ctx.config = config;

	if (!spi_dw_is_slave(spi)) {
		clear_bit_ssienr(dev);

		/* Baud rate and Slave select, for master only */
		write_baudr(dev, SPI_DW_CLK_DIVIDER(info->clock_frequency,
						    config->frequency));
	}

	if (spi_dw_is_slave(spi)) {
		LOG_DBG("Installed slave config %p:"
			    " ws/dfs %u/%u, mode %u/%u/%u",
			    config,
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0);
	} else {
		LOG_DBG("Installed master config %p: freq %uHz (div = %u),"
			    " ws/dfs %u/%u, mode %u/%u/%u, slave %u",
			    config, config->frequency,
			    SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency),
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0,
			    config->slave);
	}

	return 0;
}

static uint32_t spi_dw_compute_ndf(const struct spi_buf *rx_bufs,
				   size_t rx_count, uint8_t dfs)
{
	uint32_t len = 0U;

	for (; rx_count; rx_bufs++, rx_count--) {
		if (len > (UINT16_MAX - rx_bufs->len)) {
			goto error;
		}

		len += rx_bufs->len;
	}

	if (len) {
		return (len / dfs) - 1;
	}
error:
	return UINT32_MAX;
}

static void spi_dw_update_txftlr(const struct device *dev,
				 struct spi_dw_data *spi)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t reg_data = dw_spi_txftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (!spi->ctx.tx_len) {
			reg_data = 0U;
		} else if (spi->ctx.tx_len < dw_spi_txftlr_dflt) {
			reg_data = spi->ctx.tx_len - 1;
		}
	} else {
#if defined(CONFIG_SPI_DW_HSSI) && defined(CONFIG_SPI_EXTENDED_MODES)
		/*
		 * TXFTLR field in the TXFTLR register is valid only for
		 * Controller mode operation
		 */
		if (!spi->ctx.tx_len) {
			reg_data = 0U;
		} else if (spi->ctx.tx_len < dw_spi_txftlr_dflt) {
			reg_data = (spi->ctx.tx_len - 1) << DW_SPI_TXFTLR_TXFTLR_SHIFT;
		}
#endif
	}

	LOG_DBG("TxFTLR: %u", reg_data);

	write_txftlr(dev, reg_data);
}

static int spi_dw_xfer_polling_write_burst(const struct device *dev,
				       const uint8_t *tx_buf,
				       size_t len)
{
	const struct spi_dw_config *info = dev->config;
	const uint32_t timeout_us = CONFIG_SPI_DW_POLLING_TIMEOUT_US;
	const int64_t start = k_cycle_get_64();
	const int64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);

	const uint32_t fifo_depth = info->fifo_depth;
	size_t tx_issued = 0U;
	uint32_t iter = 0U;
	const uint32_t timeout_check_period = 64U;

	if ((tx_buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	/* Initial FIFO prime */
	{
		uint32_t n = MIN((uint32_t)len, fifo_depth);

		while (n--) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
		}
	}

	/* Refill TX FIFO until all bytes are issued */
	while (tx_issued < len) {
		uint32_t txflr = read_txflr(dev);
		uint32_t space = fifo_depth - txflr;
		uint32_t n = MIN(space, (uint32_t)(len - tx_issued));

		while (n--) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
		}

		iter++;
		if ((iter % timeout_check_period) == 0U) {
			if ((k_cycle_get_64() - start) >= timeout_cycles) {
				LOG_ERR("spi_dw timeout (%u us), tx_issued=%u len=%u",
					timeout_us,
					(uint32_t)tx_issued,
					(uint32_t)len);
				return -ETIMEDOUT;
			}
		}
	}

	/*
	* Wait until the controller finishes shifting out all queued data.
	* This path is used for burst write-only transfers.
	*/
	while (test_bit_sr_busy(dev) || (read_txflr(dev) > 0U)) {
		if ((k_cycle_get_64() - start) >= timeout_cycles) {
			LOG_ERR("spi_dw busy timeout (%u us), txflr=%u",
				timeout_us,
				read_txflr(dev));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int spi_dw_xfer_polling(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	const uint32_t timeout_us = CONFIG_SPI_DW_POLLING_TIMEOUT_US;
	const int64_t start = k_cycle_get_64();
	const int64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);

	/*
	 * Drain RX periodically instead of on every loop iteration.
	 *
	 * The goal is to keep the TX path prioritized while still preventing
	 * RX FIFO from growing too much in full-duplex transfers.
	 */
	const uint32_t rx_drain_period = 8U;
	const uint32_t timeout_check_period = 64U;

	uint32_t iter = 0U;

	ARG_UNUSED(info);

	/*
	 * Prime the TX FIFO first.
	 *
	 * This gives the polling path a TX-first behavior from the beginning,
	 * which is important for large contiguous full-duplex transfers.
	 */
	if (spi_context_tx_on(&spi->ctx) || spi_context_rx_on(&spi->ctx)) {
		push_data(dev);
	}

	while (true) {
		bool tx_on;
		bool rx_on;

		tx_on = spi_context_tx_on(&spi->ctx);
		rx_on = spi_context_rx_on(&spi->ctx);

		/*
		 * Step 1:
		 * Refill TX first so the controller can keep shifting data out
		 * as continuously as possible.
		 */
		if (tx_on || rx_on) {
			push_data(dev);
		}

		/*
		 * Step 2:
		 * Drain RX periodically during the main transfer phase.
		 *
		 * This reduces RX handling overhead compared to draining on every
		 * iteration, while still protecting against RX FIFO overrun.
		 */
		if (rx_on && ((iter % rx_drain_period) == 0U)) {
			pull_data(dev);
		}

		/*
		 * Refresh state after push/pull activity.
		 */
		tx_on = spi_context_tx_on(&spi->ctx);
		rx_on = spi_context_rx_on(&spi->ctx);

		/*
		 * Step 3:
		 * Once TX is no longer active, drain RX more aggressively so the
		 * transfer can fully retire and reach a clean completion state.
		 */
		if (!tx_on && rx_on) {
			pull_data(dev);

			tx_on = spi_context_tx_on(&spi->ctx);
			rx_on = spi_context_rx_on(&spi->ctx);
		}

		/*
		 * Completion condition:
		 * - no more TX pending in the SPI context
		 * - no more RX pending in the SPI context
		 * - no outstanding in-flight frame count
		 * - RX FIFO is empty
		 * - controller is no longer busy
		 */
		if (!tx_on &&
		    !rx_on &&
		    (spi->fifo_diff == 0U) &&
		    (read_rxflr(dev) == 0U) &&
		    !test_bit_sr_busy(dev)) {
			return 0;
		}

		iter++;

		if ((iter % timeout_check_period) == 0U) {
			if ((k_cycle_get_64() - start) >= timeout_cycles) {
				LOG_ERR("spi_dw polling timeout (%u us)", timeout_us);
				return -ETIMEDOUT;
			}
		}
	}
}

static size_t spi_buf_set_total_len(const struct spi_buf_set *bufs)
{
	size_t total = 0U;

	if ((bufs == NULL) || (bufs->buffers == NULL)) {
		return 0U;
	}

	for (size_t i = 0; i < bufs->count; i++) {
		total += bufs->buffers[i].len;
	}

	return total;
}

#ifdef CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER
/*
 * Return true when the SPI transaction matches the contiguous stream
 * transfer shape used by the OA-TC6 credit-based data path.
 *
 * The initial version only accepts one linear TX buffer and one linear
 * RX buffer with the same length, using 8-bit data frames.
 */
static bool spi_dw_can_use_stream_path(const struct device *dev,
				       const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	struct spi_dw_data *spi = dev->data;

	ARG_UNUSED(config);

	if ((tx_bufs == NULL) || (rx_bufs == NULL)) {
		return false;
	}

	if ((tx_bufs->buffers == NULL) || (rx_bufs->buffers == NULL)) {
		return false;
	}

	if ((tx_bufs->count != 1U) || (rx_bufs->count != 1U)) {
		return false;
	}

	if ((tx_bufs->buffers[0].buf == NULL) || (rx_bufs->buffers[0].buf == NULL)) {
		return false;
	}

	if ((tx_bufs->buffers[0].len == 0U) || (rx_bufs->buffers[0].len == 0U)) {
		return false;
	}

	if (tx_bufs->buffers[0].len != rx_bufs->buffers[0].len) {
		return false;
	}

	/*
	 * The first stream path is intended for byte-wise OA-TC6 transfers.
	 * Restrict the fast path to 8-bit data frames for now.
	 */
	if (spi->dfs != 1U) {
		return false;
	}

	return true;
}
#endif /* CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER */

#ifdef CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER
int spi_stream_runtime_set_tx_chunks(const struct device *dev, uint16_t tx_chunks)
{
	struct spi_dw_data *spi;

	if (dev == NULL) {
		return -EINVAL;
	}

	spi = dev->data;
	if (spi == NULL) {
		return -EINVAL;
	}

	spi->stream_tx_chunks = tx_chunks;
	spi->stream_tx_chunks_valid = true;

	return 0;
}

bool spi_stream_runtime_get_tx_chunks(const struct device *dev, uint16_t *tx_chunks)
{
	struct spi_dw_data *spi;

	if ((dev == NULL) || (tx_chunks == NULL)) {
		return false;
	}

	spi = dev->data;
	if ((spi == NULL) || !spi->stream_tx_chunks_valid) {
		return false;
	}

	*tx_chunks = spi->stream_tx_chunks;
	return true;
}

int spi_stream_runtime_set_rx_chunks(const struct device *dev, uint16_t rx_chunks)
{
	struct spi_dw_data *spi;

	if (dev == NULL) {
		return -EINVAL;
	}

	spi = dev->data;
	if (spi == NULL) {
		return -EINVAL;
	}

	spi->stream_rx_chunks = rx_chunks;
	spi->stream_rx_chunks_valid = true;

	return 0;
}

bool spi_stream_runtime_get_rx_chunks(const struct device *dev, uint16_t *rx_chunks)
{
	struct spi_dw_data *spi;

	if ((dev == NULL) || (rx_chunks == NULL)) {
		return false;
	}

	spi = dev->data;
	if ((spi == NULL) || !spi->stream_rx_chunks_valid) {
		return false;
	}

	*rx_chunks = spi->stream_rx_chunks;
	return true;
}

void spi_stream_runtime_clear(const struct device *dev)
{
	struct spi_dw_data *spi;

	if (dev == NULL) {
		return;
	}

	spi = dev->data;
	if (spi == NULL) {
		return;
	}

	spi->stream_tx_chunks = 0U;
	spi->stream_tx_chunks_valid = false;

	spi->stream_rx_chunks = 0U;
	spi->stream_rx_chunks_valid = false;
}

static int spi_dw_xfer_polling_stream8_txburst(const struct device *dev,
					       const struct spi_config *config,
					       const struct spi_buf_set *tx_bufs,
					       const struct spi_buf_set *rx_bufs)
{
	const struct spi_dw_config *info = dev->config;
	const uint32_t timeout_us = CONFIG_SPI_DW_POLLING_TIMEOUT_US;
	const int64_t start = k_cycle_get_64();
	const int64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);
	const uint32_t fifo_depth = info->fifo_depth;
	const uint32_t timeout_check_period = 64U;

	const struct spi_buf *txb;
	const struct spi_buf *rxb;
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t len;
	size_t tx_issued = 0U;
	size_t rx_done = 0U;
	uint32_t iter = 0U;

	/*
	 * Periodic RX drain budget.
	 *
	 * Change this value between 2 / 4 / 8 while testing.
	 * The earlier experiments showed that a small periodic RX service
	 * preserved stronger TX burst behavior than the more aggressive
	 * stream completion logic.
	 */
	static const uint32_t rx_periodic_quota = 1U;

	ARG_UNUSED(config);

	if ((tx_bufs == NULL) || (rx_bufs == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->buffers == NULL) || (rx_bufs->buffers == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->count != 1U) || (rx_bufs->count != 1U)) {
		return -EINVAL;
	}

	txb = &tx_bufs->buffers[0];
	rxb = &rx_bufs->buffers[0];

	if ((txb->buf == NULL) || (rxb->buf == NULL)) {
		return -EINVAL;
	}

	if ((txb->len == 0U) || (rxb->len == 0U)) {
		return -EINVAL;
	}

	if (txb->len != rxb->len) {
		return -EINVAL;
	}

	tx_buf = (const uint8_t *)txb->buf;
	rx_buf = (uint8_t *)rxb->buf;
	len = txb->len;

	/*
	 * TX-heavy fast path based on periodic RX service.
	 *
	 * Main policy:
	 *  - issue TX aggressively like the old write-burst path
	 *  - periodically drain only a very small RX amount
	 *  - at the end, empty the RX FIFO so that the next control/status
	 *    transfer does not see stale returned data
	 */

	/* Initial FIFO prime */
	{
		uint32_t n = MIN((uint32_t)len, fifo_depth);

		while (n--) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
		}
	}

	/* Refill TX FIFO until all bytes are issued */
	while (tx_issued < len) {
		uint32_t txflr = read_txflr(dev);
		uint32_t space = fifo_depth - txflr;
		uint32_t n = MIN(space, (uint32_t)(len - tx_issued));

		while (n--) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
		}

		/*
		 * Periodic RX drain.
		 *
		 * Keep this intentionally small so that TX remains dominant.
		 */
		{
			uint32_t drain = rx_periodic_quota;

			while ((drain > 0U) &&
			       (read_rxflr(dev) > 0U) &&
			       (rx_done < len)) {
				rx_buf[rx_done] = (uint8_t)read_dr(dev);
				rx_done++;
				drain--;
			}
		}

		iter++;
		if ((iter % timeout_check_period) == 0U) {
			if ((k_cycle_get_64() - start) >= timeout_cycles) {
				LOG_ERR("spi_dw txburst timeout (%u us), tx_issued=%u rx_done=%u len=%u txflr=%u rxflr=%u busy=%u",
					timeout_us,
					(uint32_t)tx_issued,
					(uint32_t)rx_done,
					(uint32_t)len,
					read_txflr(dev),
					read_rxflr(dev),
					test_bit_sr_busy(dev));
				return -ETIMEDOUT;
			}
		}
	}

	/*
	 * Tail phase.
	 *
	 * While TX is still draining, keep the same light RX service policy.
	 */
	while (test_bit_sr_busy(dev) || (read_txflr(dev) > 0U)) {
		uint32_t drain = rx_periodic_quota;

		while ((drain > 0U) &&
		       (read_rxflr(dev) > 0U) &&
		       (rx_done < len)) {
			rx_buf[rx_done] = (uint8_t)read_dr(dev);
			rx_done++;
			drain--;
		}

		if ((k_cycle_get_64() - start) >= timeout_cycles) {
			LOG_ERR("spi_dw txburst busy timeout (%u us), tx_issued=%u rx_done=%u len=%u txflr=%u rxflr=%u",
				timeout_us,
				(uint32_t)tx_issued,
				(uint32_t)rx_done,
				(uint32_t)len,
				read_txflr(dev),
				read_rxflr(dev));
			return -ETIMEDOUT;
		}
	}

	/*
	 * Final cleanup.
	 *
	 * Do not wait for full rx_done == len completion forever, but empty
	 * the RX FIFO before returning so that the next control/status
	 * transfer does not observe stale returned bytes.
	 */
	while ((read_rxflr(dev) > 0U) && (rx_done < len)) {
		rx_buf[rx_done] = (uint8_t)read_dr(dev);
		rx_done++;
	}

	while (read_rxflr(dev) > 0U) {
		(void)read_dr(dev);
	}

	return 0;
}

/*
 * RX burst path for polling-mode SPI transfers.
 *
 * This function is used when the upper protocol layer wants to harvest RX
 * data by sending dummy/empty TX bytes. Even though the purpose is RX
 * collection, the policy of this driver remains TX-preferred.
 *
 * Therefore, this path keeps feeding TX whenever the TX FIFO has room, but
 * it still drains RX often enough to avoid RX FIFO overflow. In other words,
 * RX service is performed as a safety and harvesting operation, not as a
 * scheduling priority over TX.
 *
 * The SPI controller does not know anything about OA-TC6 framing. It only
 * moves bytes between TX/RX FIFOs. Header/footer parsing, credit handling,
 * RCA interpretation, and packet assembly must stay in the protocol layer.
 */
static int spi_dw_xfer_polling_stream8_rxburst(
	const struct device *dev,
	const struct spi_config *config,
	const struct spi_buf_set *tx_bufs,
	const struct spi_buf_set *rx_bufs)
{
	const struct spi_dw_config *info = dev->config;
	const uint32_t timeout_us = CONFIG_SPI_DW_POLLING_TIMEOUT_US;
	const int64_t start = k_cycle_get_64();
	const int64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);
	const uint32_t fifo_depth = info->fifo_depth;
	const uint32_t timeout_check_period = 2048U;

	/*
	 * RX burst must be footer-safe.
	 *
	 * TX burst may minimize RX reads because returned RX data is not
	 * trusted by upper layers. RX burst is different: OA-TC6 parses
	 * returned footers from rx_buf, so every read_dr() must be guarded
	 * by RXFLR.
	 */
	const uint32_t tx_quota = 16U;
	const uint32_t rx_quota = 16U;

	const struct spi_buf *txb;
	const struct spi_buf *rxb;
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t len;
	size_t tx_issued = 0U;
	size_t rx_done = 0U;
	uint32_t iter = 0U;
	uint32_t rx_guard;

	ARG_UNUSED(config);

	if ((tx_bufs == NULL) || (rx_bufs == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->buffers == NULL) || (rx_bufs->buffers == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->count != 1U) || (rx_bufs->count != 1U)) {
		return -EINVAL;
	}

	txb = &tx_bufs->buffers[0];
	rxb = &rx_bufs->buffers[0];

	if ((txb->buf == NULL) || (rxb->buf == NULL)) {
		return -EINVAL;
	}

	if ((txb->len == 0U) || (rxb->len == 0U)) {
		return -EINVAL;
	}

	if (txb->len != rxb->len) {
		return -EINVAL;
	}

	tx_buf = (const uint8_t *)txb->buf;
	rx_buf = (uint8_t *)rxb->buf;
	len = txb->len;

	if (fifo_depth <= 4U) {
		rx_guard = fifo_depth - 1U;
	} else {
		rx_guard = fifo_depth - 2U;
	}

	/*
	 * Initial prime.
	 *
	 * Keep this below the RX FIFO guard because every TX byte produces
	 * one RX byte in full-duplex SPI.
	 */
	{
		uint32_t n = MIN(rx_guard, (uint32_t)len);

		while (n > 0U) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
			n--;
		}
	}

	while (rx_done < len) {
		uint32_t outstanding;
		uint32_t rxflr;
		uint32_t n;

		/*
		 * RX service.
		 *
		 * Never read DR unless RXFLR says data is available.
		 * This is required to keep OA-TC6 footer alignment valid.
		 */
		rxflr = read_rxflr(dev);
		if ((rxflr > 0U) && (rx_done < tx_issued)) {
			n = MIN(rxflr, rx_quota);
			n = MIN(n, (uint32_t)(tx_issued - rx_done));
			n = MIN(n, (uint32_t)(len - rx_done));

			while (n > 0U) {
				rx_buf[rx_done] = (uint8_t)read_dr(dev);
				rx_done++;
				n--;
			}
		}

		/*
		 * TX refill.
		 *
		 * We still avoid TXFLR in the hot path, but use the
		 * outstanding byte count to prevent RX FIFO overflow.
		 */
		if (tx_issued < len) {
			outstanding = (uint32_t)(tx_issued - rx_done);

			if (outstanding < rx_guard) {
				n = rx_guard - outstanding;
				n = MIN(n, tx_quota);
				n = MIN(n, (uint32_t)(len - tx_issued));

				while (n > 0U) {
					write_dr(dev, tx_buf[tx_issued]);
					tx_issued++;
					n--;
				}
			}
		}

		/*
		 * Tail phase.
		 *
		 * Once all TX bytes are issued, keep waiting for RXFLR and
		 * drain until rx_done == len. Do not read DR blindly.
		 */
		if (tx_issued >= len) {
			while (rx_done < len) {
				rxflr = read_rxflr(dev);

				if (rxflr > 0U) {
					n = MIN(rxflr, rx_quota);
					n = MIN(n, (uint32_t)(len - rx_done));

					while (n > 0U) {
						rx_buf[rx_done] = (uint8_t)read_dr(dev);
						rx_done++;
						n--;
					}

					continue;
				}

				if ((k_cycle_get_64() - start) >= timeout_cycles) {
					LOG_ERR("spi_dw rxburst tail timeout (%u us), tx_issued=%u rx_done=%u len=%u rxflr=%u busy=%u",
						timeout_us,
						(uint32_t)tx_issued,
						(uint32_t)rx_done,
						(uint32_t)len,
						read_rxflr(dev),
						test_bit_sr_busy(dev));
					return -ETIMEDOUT;
				}
			}
		}

		iter++;
		if ((iter % timeout_check_period) == 0U) {
			if ((k_cycle_get_64() - start) >= timeout_cycles) {
				LOG_ERR("spi_dw rxburst timeout (%u us), tx_issued=%u rx_done=%u len=%u outstanding=%u rxflr=%u busy=%u",
					timeout_us,
					(uint32_t)tx_issued,
					(uint32_t)rx_done,
					(uint32_t)len,
					(uint32_t)(tx_issued - rx_done),
					read_rxflr(dev),
					test_bit_sr_busy(dev));
				return -ETIMEDOUT;
			}
		}
	}

	/*
	 * Defensive cleanup.
	 *
	 * Normally rx_done == len means the transaction is complete.
	 * If any stale RX byte remains, drain it explicitly.
	 */
	while (read_rxflr(dev) > 0U) {
		(void)read_dr(dev);
	}

	return 0;
}

/*
 * Polling-based contiguous full-duplex transfer path for 8-bit frames.
 *
 * This helper is intended for protocol layers such as OA-TC6 that build
 * one linear TX/RX buffer pair and need to execute it as one logical SPI
 * data transaction with low software overhead.
 *
 * The initial implementation only accepts one TX buffer and one RX buffer
 * with the same length and assumes 8-bit data frames.
 */
static int spi_dw_xfer_polling_stream8_default(
	const struct device *dev,
	const struct spi_config *config,
	const struct spi_buf_set *tx_bufs,
	const struct spi_buf_set *rx_bufs)
{
	const struct spi_dw_config *info = dev->config;
	const uint32_t timeout_us = CONFIG_SPI_DW_POLLING_TIMEOUT_US;
	const int64_t start = k_cycle_get_64();
	const int64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);
	const uint32_t fifo_depth = info->fifo_depth;
	const uint32_t timeout_check_period = 64U;

	const struct spi_buf *txb;
	const struct spi_buf *rxb;
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t len;
	size_t tx_issued = 0U;
	size_t rx_done = 0U;
	uint32_t iter = 0U;
	uint32_t rx_hi_wm;
	uint32_t rx_lo_wm;

	ARG_UNUSED(config);

	if ((tx_bufs == NULL) || (rx_bufs == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->buffers == NULL) || (rx_bufs->buffers == NULL)) {
		return -EINVAL;
	}

	if ((tx_bufs->count != 1U) || (rx_bufs->count != 1U)) {
		return -EINVAL;
	}

	txb = &tx_bufs->buffers[0];
	rxb = &rx_bufs->buffers[0];

	if ((txb->buf == NULL) || (rxb->buf == NULL)) {
		return -EINVAL;
	}

	if ((txb->len == 0U) || (rxb->len == 0U)) {
		return -EINVAL;
	}

	if (txb->len != rxb->len) {
		return -EINVAL;
	}

	tx_buf = (const uint8_t *)txb->buf;
	rx_buf = (uint8_t *)rxb->buf;
	len = txb->len;

	/*
	 * Default / generic full-duplex path.
	 *
	 * This path intentionally keeps stronger RX service for:
	 * - control transfers
	 * - small transfers
	 * - generic full-duplex operation
	 */
	rx_hi_wm = (fifo_depth > 4U) ? (fifo_depth / 2U) : 1U;
	rx_lo_wm = (fifo_depth > 8U) ? (fifo_depth / 4U) : 1U;

	/*
	 * Initial FIFO prime.
	 */
	{
		uint32_t n = MIN((uint32_t)len, fifo_depth);

		while (n > 0U) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
			n--;
		}
	}

	while (tx_issued < len) {
		uint32_t txflr = read_txflr(dev);
		uint32_t space = fifo_depth - txflr;
		uint32_t n = MIN(space, (uint32_t)(len - tx_issued));

		while (n > 0U) {
			write_dr(dev, tx_buf[tx_issued]);
			tx_issued++;
			n--;
		}

		if ((read_rxflr(dev) >= rx_hi_wm) && (rx_done < tx_issued)) {
			do {
				rx_buf[rx_done] = (uint8_t)read_dr(dev);
				rx_done++;
			} while ((read_rxflr(dev) > rx_lo_wm) &&
				 (rx_done < tx_issued));
		}

		iter++;
		if ((iter % timeout_check_period) == 0U) {
			if ((k_cycle_get_64() - start) >= timeout_cycles) {
				LOG_ERR("spi_dw default timeout (%u us), tx_issued=%u rx_done=%u len=%u txflr=%u rxflr=%u busy=%u",
					timeout_us,
					(uint32_t)tx_issued,
					(uint32_t)rx_done,
					(uint32_t)len,
					read_txflr(dev),
					read_rxflr(dev),
					test_bit_sr_busy(dev));
				return -ETIMEDOUT;
			}
		}
	}

	/*
	 * Wait until TX FIFO/shifter is drained.
	 * Keep RX serviced while waiting.
	 */
	while (test_bit_sr_busy(dev) || (read_txflr(dev) > 0U)) {
		while ((read_rxflr(dev) > 0U) && (rx_done < tx_issued)) {
			rx_buf[rx_done] = (uint8_t)read_dr(dev);
			rx_done++;
		}

		if ((k_cycle_get_64() - start) >= timeout_cycles) {
			LOG_ERR("spi_dw default busy timeout (%u us), tx_issued=%u rx_done=%u len=%u txflr=%u rxflr=%u busy=%u",
				timeout_us,
				(uint32_t)tx_issued,
				(uint32_t)rx_done,
				(uint32_t)len,
				read_txflr(dev),
				read_rxflr(dev),
				test_bit_sr_busy(dev));
			return -ETIMEDOUT;
		}
	}

	/*
	 * Complete RX.
	 *
	 * Unlike txburst/rxburst paths, the default path remains conservative:
	 * it waits until all expected RX bytes are consumed.
	 */
	while (rx_done < len) {
		while ((rx_done < len) && (read_rxflr(dev) > 0U)) {
			rx_buf[rx_done] = (uint8_t)read_dr(dev);
			rx_done++;
		}

		if ((k_cycle_get_64() - start) >= timeout_cycles) {
			LOG_ERR("spi_dw default rx timeout (%u us), tx_issued=%u rx_done=%u len=%u txflr=%u rxflr=%u busy=%u",
				timeout_us,
				(uint32_t)tx_issued,
				(uint32_t)rx_done,
				(uint32_t)len,
				read_txflr(dev),
				read_rxflr(dev),
				test_bit_sr_busy(dev));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int spi_dw_xfer_polling_stream8(const struct device *dev,
				       const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	uint16_t tx_chunks = 0U;
	uint16_t rx_chunks = 0U;
	bool has_tx_chunks;
	bool has_rx_chunks;

	has_tx_chunks = spi_stream_runtime_get_tx_chunks(dev, &tx_chunks);
	has_rx_chunks = spi_stream_runtime_get_rx_chunks(dev, &rx_chunks);

	/*
	 * TX-preferred dispatch policy.
	 *
	 * Real TX payload always has priority over RX harvesting. RX burst is
	 * selected only when there is no TX burst request and the upper layer
	 * explicitly marks the next transfer as an RX-harvest transfer.
	 */
	/* TX first policy */
	if (has_tx_chunks && (tx_chunks >= 16U)) {
		return spi_dw_xfer_polling_stream8_txburst(dev, config,
							   tx_bufs, rx_bufs);
	}

	/* RX burst path */
	if (has_rx_chunks && (rx_chunks > 0U)) {
		return spi_dw_xfer_polling_stream8_rxburst(dev, config,
							   tx_bufs, rx_bufs);
	}

	/* fallback */
	return spi_dw_xfer_polling_stream8_default(dev, config,
						   tx_bufs, rx_bufs);
}

#endif /* CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER */

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t dw_spi_rxftlr_dflt = (info->fifo_depth * 5) / 8;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure(dev, spi, config);
	if (ret) {
		goto out;
	}

	/* controller disabled while programming registers */
	if (!spi_dw_is_slave(spi)) {
		clear_bit_ssienr(dev);
	}

	/* Determine TMOD */
	if (!rx_bufs || !rx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_TX;
	} else if (!tx_bufs || !tx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_RX;
	}

	/* ToDo: add a way to determine EEPROM mode */

	if (tmod >= DW_SPI_CTRLR0_TMOD_RX &&
	    !spi_dw_is_slave(spi)) {
		reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
		if (reg_data == UINT32_MAX) {
			ret = -EINVAL;
			goto out;
		}

		write_ctrlr1(dev, reg_data);
	} else {
		write_ctrlr1(dev, 0);
	}

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
		if (tmod == DW_SPI_CTRLR0_TMOD_RX) {
			tmod |= DW_SPI_CTRLR0_SLV_OE;
		} else {
			tmod &= ~DW_SPI_CTRLR0_SLV_OE;
		}
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);

#ifdef CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER
	/*
	 * Use the polling-based contiguous stream path when the transfer
	 * matches the OA-TC6 stream transaction shape.
	 *
	 * This path is only used for synchronous polling transfers and
	 * falls back to the regular SPI context-based path otherwise.
	 */
	if (!asynchronous &&
	    !IS_ENABLED(CONFIG_SPI_DW_USE_IRQ) &&
	    spi_dw_can_use_stream_path(dev, config, tx_bufs, rx_bufs)) {
		if (!spi_dw_is_slave(spi)) {
			/* if cs is not defined as gpio, use hw cs */
			write_ser(dev, BIT(config->slave));
			if (spi_cs_is_gpio(config)) {
				spi_context_cs_control(&spi->ctx, true);
			}
		}

		LOG_DBG("Enabling controller for stream transfer");
		set_bit_ssienr(dev);

		ret = spi_dw_xfer_polling_stream8(dev, config, tx_bufs, rx_bufs);
		completed(dev, ret);
		goto out;
	}
#endif /* CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER */

	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);

	spi->fifo_diff = 0U;

	/* Tx Threshold */
	spi_dw_update_txftlr(dev, spi);

	/* Does Rx thresholds needs to be lower? */
	reg_data = dw_spi_rxftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (spi->ctx.rx_len &&
		    spi->ctx.rx_len < dw_spi_rxftlr_dflt) {
			reg_data = spi->ctx.rx_len - 1;
		}
	} else {
		if (spi->ctx.rx_len && spi->ctx.rx_len < info->fifo_depth) {
			reg_data = spi->ctx.rx_len - 1;
		}
	}

	/* Rx Threshold */
	write_rxftlr(dev, reg_data);

	/* Enable interrupts */
	if (IS_ENABLED(CONFIG_SPI_DW_USE_IRQ)) {
		reg_data = !rx_bufs ?
			(DW_SPI_IMR_UNMASK & DW_SPI_IMR_MASK_RX) :
			DW_SPI_IMR_UNMASK;
	} else {
		reg_data = DW_SPI_IMR_MASK;
	}
	write_imr(dev, reg_data);

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		write_ser(dev, BIT(config->slave));
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
		}
	}

	LOG_DBG("Enabling controller");
	set_bit_ssienr(dev);

	if (IS_ENABLED(CONFIG_SPI_DW_USE_IRQ)) {
		ret = spi_context_wait_for_completion(&spi->ctx);
	} else {
		ret = spi_dw_xfer_polling(dev);
		completed(dev, ret);
	}
#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&spi->ctx) && !ret) {
		ret = spi->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

out:
	spi_context_release(&spi->ctx, ret);

	if (!spi_dw_is_slave(spi)) {
		write_ser(dev, 0);
	}

	pm_device_busy_clear(dev);

	return ret;
}

static int spi_dw_transceive(const struct device *dev,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_dw_transceive_async(const struct device *dev,
				   const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs,
				   spi_callback_t cb,
				   void *userdata)
{
	LOG_DBG("%p, %p, %p, %p, %p", dev, tx_bufs, rx_bufs, cb, userdata);

	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_dw_release(const struct device *dev,
			  const struct spi_config *config)
{
	struct spi_dw_data *spi = dev->data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

void spi_dw_isr(const struct device *dev)
{
	uint32_t int_status;
	int error;

	int_status = read_isr(dev);

	LOG_DBG("SPI %p int_status 0x%x - (tx: %d, rx: %d)", dev, int_status,
		read_txflr(dev), read_rxflr(dev));

	if (int_status & DW_SPI_ISR_ERRORS_MASK) {
		error = -EIO;
		goto out;
	}

	error = 0;

	if (int_status & DW_SPI_ISR_RXFIS) {
		pull_data(dev);
	}

	if (int_status & DW_SPI_ISR_TXEIS) {
		push_data(dev);
	}

out:
	clear_interrupts(dev);
	completed(dev, error);
}

static DEVICE_API(spi, dw_spi_api) = {
	.transceive = spi_dw_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_dw_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_dw_release,
};

int spi_dw_init(const struct device *dev)
{
	int err;
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

#ifdef CONFIG_PINCTRL
	pinctrl_apply_state(info->pcfg, PINCTRL_STATE_DEFAULT);
#endif

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* IRQ mode only */
	if (info->config_func) {
		info->config_func();
	}

	/* Masking interrupt and making sure controller is disabled */
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_bit_ssienr(dev);

#if !DT_ANY_INST_PROP_STATUS_OKAY(aux_reg)
	/* SSI component version */
	spi->version = read_ssi_comp_version(dev);
	LOG_DBG("Version: %c.%c%c%c", (spi->version >> 24) & 0xff,
		(spi->version >> 16) & 0xff, (spi->version >> 8) & 0xff,
		spi->version & 0xff);
#endif

	LOG_DBG("Designware SPI driver initialized on device: %p", dev);

	err = spi_context_cs_configure_all(&spi->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

#define SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, err_int, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, err_int, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, err_int, irq));

#define SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, txo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, txo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxu_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, mst_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, mst_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, txo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, mst_err, irq));

#define SPI_DW_IRQ_HANDLER(inst)                                   \
void spi_dw_irq_config_##inst(void)                                \
{                                                                  \
COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 1),              \
	(IRQ_CONNECT(DT_INST_IRQN(inst),                           \
		DT_INST_IRQ(inst, priority),                       \
		spi_dw_isr, DEVICE_DT_INST_GET(inst),              \
		0);                                                \
	irq_enable(DT_INST_IRQN(inst));),                          \
	(COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 3),     \
		(SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)),		   \
		(SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)))))	   \
}

#if defined(CONFIG_CLOCK_CONTROL)
#define CLOCK_DW_CONFIG(n)                                                             \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(0, clocks),                                   \
		   (.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                  \
		    .clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clkid),))
#else
#define CLOCK_DW_CONFIG(n)
#endif

#if defined(CONFIG_SPI_DW_USE_IRQ)
#define SPI_DW_INIT(inst)                                                                   \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                         \
	SPI_DW_IRQ_HANDLER(inst);                                                           \
	static struct spi_dw_data spi_dw_data_##inst = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_INIT_SYNC(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                     \
	};                                                                                  \
	static const struct spi_dw_config spi_dw_config_##inst = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                    \
		.clock_frequency = COND_CODE_1(                                             \
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),   \
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),           \
			(DT_INST_PROP(inst, clock_frequency))),                             \
		.config_func = spi_dw_irq_config_##inst,                                    \
		.serial_target = DT_INST_PROP(inst, serial_target),                         \
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),                               \
		.max_xfer_size = DT_INST_PROP(inst, max_xfer_size),                         \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),)) \
		COND_CODE_1(DT_INST_PROP(inst, aux_reg),                                    \
			(.read_func = aux_reg_read,                                         \
			.write_func = aux_reg_write,                                        \
			.set_bit_func = aux_reg_set_bit,                                    \
			.clear_bit_func = aux_reg_clear_bit,                                \
			.test_bit_func = aux_reg_test_bit,),                                \
			(.read_func = reg_read,                                             \
			.write_func = reg_write,                                            \
			.set_bit_func = reg_set_bit,                                        \
			.clear_bit_func = reg_clear_bit,                                    \
			.test_bit_func = reg_test_bit,))                                    \
	};                                                                                  \
	SPI_DEVICE_DT_INST_DEFINE(inst,                                                     \
		spi_dw_init,                                                                \
		NULL,                                                                       \
		&spi_dw_data_##inst,                                                        \
		&spi_dw_config_##inst,                                                      \
		POST_KERNEL,                                                                \
		CONFIG_SPI_INIT_PRIORITY,                                                   \
		&dw_spi_api);

#else /* !CONFIG_SPI_DW_USE_IRQ */

#define SPI_DW_INIT(inst)                                                                   \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                         \
	static struct spi_dw_data spi_dw_data_##inst = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_INIT_SYNC(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                     \
	};                                                                                  \
	static const struct spi_dw_config spi_dw_config_##inst = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                    \
		.clock_frequency = COND_CODE_1(                                             \
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),   \
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),           \
			(DT_INST_PROP(inst, clock_frequency))),                             \
		.config_func = NULL,                                                        \
		.serial_target = DT_INST_PROP(inst, serial_target),                         \
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),                               \
		.max_xfer_size = DT_INST_PROP(inst, max_xfer_size),                         \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),)) \
		COND_CODE_1(DT_INST_PROP(inst, aux_reg),                                    \
			(.read_func = aux_reg_read,                                         \
			 .write_func = aux_reg_write,                                        \
			 .set_bit_func = aux_reg_set_bit,                                    \
			 .clear_bit_func = aux_reg_clear_bit,                                \
			 .test_bit_func = aux_reg_test_bit,),                                \
			(.read_func = reg_read,                                             \
			 .write_func = reg_write,                                            \
			 .set_bit_func = reg_set_bit,                                        \
			 .clear_bit_func = reg_clear_bit,                                    \
			 .test_bit_func = reg_test_bit,))                                    \
		CLOCK_DW_CONFIG(inst)                                                      \
	};                                                                                  \
	SPI_DEVICE_DT_INST_DEFINE(inst,                                                     \
		spi_dw_init,                                                                \
		NULL,                                                                       \
		&spi_dw_data_##inst,                                                        \
		&spi_dw_config_##inst,                                                      \
		POST_KERNEL,                                                                \
		CONFIG_SPI_INIT_PRIORITY,                                                   \
		&dw_spi_api);

#endif /* CONFIG_SPI_DW_USE_IRQ */

DT_INST_FOREACH_STATUS_OKAY(SPI_DW_INIT)
