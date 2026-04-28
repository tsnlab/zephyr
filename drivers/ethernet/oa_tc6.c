/*
 * Copyright (c) 2023 DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/mdio.h>
#include "oa_tc6.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(oa_tc6, CONFIG_ETHERNET_LOG_LEVEL);

#define OA_TC6_XFER_CHUNK_BUDGET 32U
/*
 * Internal OA-TC6 transfer plan used by the common data transfer path.
 *
 * The initial version is TX-only. RX-related fields can be added later
 * when RX harvesting is integrated into the same planner.
 */
struct oa_tc6_xfer_plan {
	uint8_t tx_chunks;
	uint8_t rx_chunks;
	uint8_t total_chunks;
};

/*
 * When IPv6 support enabled - the minimal size of network buffer
 * shall be at least 128 bytes (i.e. default value).
 */
#if defined(CONFIG_NET_IPV6) && (CONFIG_NET_BUF_DATA_SIZE < 128)
#error IPv6 requires at least 128 bytes of continuous data to handle headers!
#endif

int oa_tc6_reg_read(struct oa_tc6 *tc6, const uint32_t reg, uint32_t *val)
{
	uint8_t buf[OA_TC6_HDR_SIZE + 12] = {0};
	struct spi_buf tx_buf = {.buf = buf, .len = sizeof(buf)};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = buf, .len = sizeof(buf)};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
	uint32_t rv, rvn, hdr_bkp, *hdr = (uint32_t *)&buf[0];
	int ret = 0;

	/*
	 * Buffers are allocated for protected (larger) case (by 4 bytes).
	 * When non-protected case - we need to decrase them
	 */
	if (!tc6->protected) {
		tx_buf.len -= sizeof(rvn);
		rx_buf.len -= sizeof(rvn);
	}

	*hdr = FIELD_PREP(OA_CTRL_HDR_DNC, 0) | FIELD_PREP(OA_CTRL_HDR_WNR, 0) |
	       FIELD_PREP(OA_CTRL_HDR_AID, 0) | FIELD_PREP(OA_CTRL_HDR_MMS, reg >> 16) |
	       FIELD_PREP(OA_CTRL_HDR_ADDR, reg) |
	       FIELD_PREP(OA_CTRL_HDR_LEN, 0); /* To read single register len = 0 */
	*hdr |= FIELD_PREP(OA_CTRL_HDR_P, oa_tc6_get_parity(*hdr));
	hdr_bkp = *hdr;
	*hdr = sys_cpu_to_be32(*hdr);

	ret = spi_transceive_dt(tc6->spi, &tx, &rx);
	if (ret < 0) {
		return ret;
	}

	/* Check if echoed control command header is correct */
	rv = sys_be32_to_cpu(*(uint32_t *)&buf[4]);
	if (hdr_bkp != rv) {
		LOG_ERR("Header transmission error!");
		return -1;
	}

	rv = sys_be32_to_cpu(*(uint32_t *)&buf[8]);

	/* In protected mode read data is followed by its compliment value */
	if (tc6->protected) {
		rvn = sys_be32_to_cpu(*(uint32_t *)&buf[12]);
		if (rv != ~rvn) {
			LOG_ERR("Protected mode transmission error!");
			return -1;
		}
	}

	*val = rv;

	return ret;
}

int oa_tc6_reg_write(struct oa_tc6 *tc6, const uint32_t reg, uint32_t val)
{
	uint8_t buf_tx[OA_TC6_HDR_SIZE + 12] = {0};
	uint8_t buf_rx[OA_TC6_HDR_SIZE + 12] = {0};
	struct spi_buf tx_buf = {.buf = buf_tx, .len = sizeof(buf_tx)};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = buf_rx, .len = sizeof(buf_rx)};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
	uint32_t rv, rvn, hdr_bkp, *hdr = (uint32_t *)&buf_tx[0];
	int ret;

	/*
	 * Buffers are allocated for protected (larger) case (by 4 bytes).
	 * When non-protected case - we need to decrase them
	 */
	if (!tc6->protected) {
		tx_buf.len -= sizeof(rvn);
		rx_buf.len -= sizeof(rvn);
	}

	*hdr = FIELD_PREP(OA_CTRL_HDR_DNC, 0) | FIELD_PREP(OA_CTRL_HDR_WNR, 1) |
	       FIELD_PREP(OA_CTRL_HDR_AID, 0) | FIELD_PREP(OA_CTRL_HDR_MMS, reg >> 16) |
	       FIELD_PREP(OA_CTRL_HDR_ADDR, reg) |
	       FIELD_PREP(OA_CTRL_HDR_LEN, 0); /* To read single register len = 0 */
	*hdr |= FIELD_PREP(OA_CTRL_HDR_P, oa_tc6_get_parity(*hdr));
	hdr_bkp = *hdr;
	*hdr = sys_cpu_to_be32(*hdr);

	*(uint32_t *)&buf_tx[4] = sys_cpu_to_be32(val);
	if (tc6->protected) {
		*(uint32_t *)&buf_tx[8] = sys_be32_to_cpu(~val);
	}

	ret = spi_transceive_dt(tc6->spi, &tx, &rx);
	if (ret < 0) {
		return ret;
	}

	/* Check if echoed control command header is correct */
	rv = sys_be32_to_cpu(*(uint32_t *)&buf_rx[4]);
	if (hdr_bkp != rv) {
		LOG_ERR("Header transmission error!");
		return -1;
	}

	/* Check if echoed value is correct */
	rv = sys_be32_to_cpu(*(uint32_t *)&buf_rx[8]);
	if (val != rv) {
		LOG_ERR("Header transmission error!");
		return -1;
	}

	/*
	 * In protected mode check if read value is followed by its
	 * compliment value
	 */
	if (tc6->protected) {
		rvn = sys_be32_to_cpu(*(uint32_t *)&buf_rx[12]);
		if (val != ~rvn) {
			LOG_ERR("Protected mode transmission error!");
			return -1;
		}
	}

	return ret;
}

int oa_tc6_reg_rmw(struct oa_tc6 *tc6, const uint32_t reg, uint32_t mask, uint32_t val)
{
	uint32_t tmp;
	int ret;

	ret = oa_tc6_reg_read(tc6, reg, &tmp);
	if (ret < 0) {
		return ret;
	}

	tmp &= ~mask;

	if (val) {
		tmp |= val;
	}

	return oa_tc6_reg_write(tc6, reg, tmp);
}

int oa_tc6_mdio_read(struct oa_tc6 *tc6, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	return oa_tc6_reg_read(
		tc6, OA_TC6_PHY_STD_REG_ADDR_BASE | (regad & OA_TC6_PHY_STD_REG_ADDR_MASK),
		(uint32_t *)data);
}

int oa_tc6_mdio_write(struct oa_tc6 *tc6, uint8_t prtad, uint8_t regad, uint16_t data)
{
	return oa_tc6_reg_write(
		tc6, OA_TC6_PHY_STD_REG_ADDR_BASE | (regad & OA_TC6_PHY_STD_REG_ADDR_MASK), data);
}

static int oa_tc6_get_phy_c45_mms(int devad)
{
	switch (devad) {
	case MDIO_MMD_PCS:
		return OA_TC6_PHY_C45_PCS_MMS2;
	case MDIO_MMD_PMAPMD:
		return OA_TC6_PHY_C45_PMA_PMD_MMS3;
	case MDIO_MMD_VENDOR_SPECIFIC2:
		return OA_TC6_PHY_C45_VS_PLCA_MMS4;
	case MDIO_MMD_AN:
		return OA_TC6_PHY_C45_AUTO_NEG_MMS5;
	default:
		return -EOPNOTSUPP;
	}
}

int oa_tc6_mdio_read_c45(struct oa_tc6 *tc6, uint8_t prtad, uint8_t devad, uint16_t regad,
			 uint16_t *data)
{
	uint32_t tmp;
	int ret;

	ret = oa_tc6_get_phy_c45_mms(devad);
	if (ret < 0) {
		return ret;
	}

	ret = oa_tc6_reg_read(tc6, (ret << 16) | regad, &tmp);
	if (ret < 0) {
		return ret;
	}

	*data = (uint16_t)tmp;

	return 0;
}

int oa_tc6_mdio_write_c45(struct oa_tc6 *tc6, uint8_t prtad, uint8_t devad, uint16_t regad,
			  uint16_t data)
{
	int ret;

	ret = oa_tc6_get_phy_c45_mms(devad);
	if (ret < 0) {
		return ret;
	}

	return oa_tc6_reg_write(tc6, (ret << 16) | regad, (uint32_t)data);
}

int oa_tc6_set_protected_ctrl(struct oa_tc6 *tc6, bool prote)
{
	int ret;

	ret = oa_tc6_reg_rmw(tc6, OA_CONFIG0, OA_CONFIG0_PROTE, prote ? OA_CONFIG0_PROTE : 0);
	if (ret < 0) {
		return ret;
	}

	tc6->protected = prote;
	return 0;
}

int oa_tc6_send_chunks(struct oa_tc6 *tc6, struct net_pkt *pkt)
{
	uint16_t len = net_pkt_get_len(pkt);
	uint8_t oa_tx[tc6->cps];
	uint32_t hdr, ftr;
	uint8_t chunks, i;
	int ret;

	if (len == 0) {
		return -ENODATA;
	}

	chunks = len / tc6->cps;
	if (len % tc6->cps) {
		chunks++;
	}

	/* Check if LAN865x has any free internal buffer space */
	if (chunks > tc6->txc) {
		return -EIO;
	}

	/* Transform struct net_pkt content into chunks */
	for (i = 1; i <= chunks; i++) {
		hdr = FIELD_PREP(OA_DATA_HDR_DNC, 1) | FIELD_PREP(OA_DATA_HDR_DV, 1) |
		      FIELD_PREP(OA_DATA_HDR_NORX, 1) | FIELD_PREP(OA_DATA_HDR_SWO, 0);

		if (i == 1) {
			hdr |= FIELD_PREP(OA_DATA_HDR_SV, 1);
		}

		if (i == chunks) {
			hdr |= FIELD_PREP(OA_DATA_HDR_EBO, len - 1) | FIELD_PREP(OA_DATA_HDR_EV, 1);
		}

		hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));

		ret = net_pkt_read(pkt, oa_tx, len > tc6->cps ? tc6->cps : len);
		if (ret < 0) {
			return ret;
		}

		ret = oa_tc6_chunk_spi_transfer(tc6, NULL, oa_tx, hdr, &ftr);
		if (ret < 0) {
			return ret;
		}

		len -= tc6->cps;
	}

	return 0;
}

int oa_tc6_check_status(struct oa_tc6 *tc6)
{
	uint32_t sts;

	if (!tc6->sync) {
		LOG_ERR("SYNC: Configuration lost, reset IC!");
		return -EIO;
	}

	if (tc6->exst) {
		/*
		 * Just clear any pending interrupts.
		 * The RESETC is handled separately as it requires per
		 * device configuration.
		 */
		if (oa_tc6_reg_read(tc6, OA_STATUS0, &sts) < 0) {
			return -EIO;
		}

		if (sts != 0) {
			oa_tc6_reg_write(tc6, OA_STATUS0, sts);
			LOG_WRN("EXST: OA_STATUS0: 0x%x", sts);
		}

		if (oa_tc6_reg_read(tc6, OA_STATUS1, &sts) < 0) {
			return -EIO;
		}

		if (sts != 0) {
			oa_tc6_reg_write(tc6, OA_STATUS1, sts);
			LOG_WRN("EXST: OA_STATUS1: 0x%x", sts);
		}
	}

	return 0;
}

static inline int oa_tc6_update_status(struct oa_tc6 *tc6, uint32_t ftr)
{
	if (oa_tc6_get_parity(ftr)) {
		LOG_DBG("OA Status Update: Footer parity error!");
		return -EIO;
	}

	tc6->exst = FIELD_GET(OA_DATA_FTR_EXST, ftr);
	tc6->sync = FIELD_GET(OA_DATA_FTR_SYNC, ftr);
	tc6->rca = FIELD_GET(OA_DATA_FTR_RCA, ftr);
	tc6->txc = FIELD_GET(OA_DATA_FTR_TXC, ftr);

	return 0;
}

int oa_tc6_chunk_spi_transfer(struct oa_tc6 *tc6, uint8_t *buf_rx, uint8_t *buf_tx, uint32_t hdr,
			      uint32_t *ftr)
{
	struct spi_buf tx_buf[2];
	struct spi_buf rx_buf[2];
	struct spi_buf_set tx;
	struct spi_buf_set rx;
	int ret;

	hdr = sys_cpu_to_be32(hdr);
	tx_buf[0].buf = &hdr;
	tx_buf[0].len = sizeof(hdr);

	tx_buf[1].buf = buf_tx;
	tx_buf[1].len = tc6->cps;

	tx.buffers = tx_buf;
	tx.count = ARRAY_SIZE(tx_buf);

	rx_buf[0].buf = buf_rx;
	rx_buf[0].len = tc6->cps;

	rx_buf[1].buf = ftr;
	rx_buf[1].len = sizeof(*ftr);

	rx.buffers = rx_buf;
	rx.count = ARRAY_SIZE(rx_buf);

	ret = spi_transceive_dt(tc6->spi, &tx, &rx);
	if (ret < 0) {
		return ret;
	}
	*ftr = sys_be32_to_cpu(*ftr);

	return oa_tc6_update_status(tc6, *ftr);
}

int oa_tc6_read_status(struct oa_tc6 *tc6, uint32_t *ftr)
{
	uint32_t hdr;

	hdr = FIELD_PREP(OA_DATA_HDR_DNC, 1) | FIELD_PREP(OA_DATA_HDR_DV, 0) |
	      FIELD_PREP(OA_DATA_HDR_NORX, 1);
	hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));

	return oa_tc6_chunk_spi_transfer(tc6, NULL, NULL, hdr, ftr);
}

/*
 * Transfer multiple OA-TC6 RX chunks in one SPI transaction.
 *
 * This function sends empty OA-TC6 data chunks to generate SPI clocks and
 * harvest RX chunks from the MAC-PHY. Each returned footer is parsed through
 * oa_tc6_update_status(), so RCA/TXC/SYNC/EXST are refreshed after the
 * transaction.
 *
 * The SPI controller does not interpret OA-TC6 data. It only transfers raw
 * bytes. Header/footer handling remains in this OA-TC6 layer.
 */
static int oa_tc6_chunk_spi_rx_transfer_burst(struct oa_tc6 *tc6,
					      uint8_t *buf_rx,
					      uint32_t *ftrs,
					      uint8_t chunks)
{
	struct spi_buf tx_buf;
	struct spi_buf rx_buf;
	struct spi_buf_set tx;
	struct spi_buf_set rx;
	uint32_t hdr;
	size_t total_len;
	int ret;
	int i;

	if ((tc6 == NULL) || (ftrs == NULL) || (chunks == 0U)) {
		return -EINVAL;
	}

	if (chunks > OA_TC6_XFER_CHUNK_BUDGET) {
		return -EINVAL;
	}

	hdr = FIELD_PREP(OA_DATA_HDR_DNC, 1);
	hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));

	for (i = 0; i < chunks; i++) {
		uint8_t *txp = &tc6->spi_data_tx_buf[i * OA_TC6_TX_CHUNK_SIZE];
		uint32_t hdr_be = sys_cpu_to_be32(hdr);

		memcpy(txp, &hdr_be, sizeof(hdr_be));
		memset(txp + sizeof(hdr_be), 0, tc6->cps);
	}

	total_len = chunks * OA_TC6_TX_CHUNK_SIZE;

	tx_buf.buf = tc6->spi_data_tx_buf;
	tx_buf.len = total_len;

	rx_buf.buf = tc6->spi_data_rx_buf;
	rx_buf.len = total_len;

	tx.buffers = &tx_buf;
	tx.count = 1;

	rx.buffers = &rx_buf;
	rx.count = 1;

	ret = spi_stream_runtime_set_rx_chunks(tc6->spi->bus, chunks);
	if (ret < 0) {
		return ret;
	}

	ret = spi_transceive_dt(tc6->spi, &tx, &rx);

	spi_stream_runtime_clear(tc6->spi->bus);

	if (ret < 0) {
		return ret;
	}

	for (i = 0; i < chunks; i++) {
		uint8_t *rxp = &tc6->spi_data_rx_buf[i * OA_TC6_RX_CHUNK_SIZE];
		uint32_t chunk_ftr;

		if (buf_rx != NULL) {
			memcpy(buf_rx + (i * tc6->cps), rxp, tc6->cps);
		}

		memcpy(&chunk_ftr, rxp + tc6->cps, sizeof(chunk_ftr));
		chunk_ftr = sys_be32_to_cpu(chunk_ftr);

		ftrs[i] = chunk_ftr;

		ret = oa_tc6_update_status(tc6, chunk_ftr);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Process one already-harvested OA-TC6 RX chunk.
 *
 * This helper should contain the existing per-chunk RX assembly logic from
 * the old oa_tc6_read_chunks() implementation. It must not perform another
 * SPI transfer. It only interprets one chunk payload and its footer, appends
 * valid frame bytes to the net_pkt, and returns 0 when one complete Ethernet
 * frame has been assembled.
 */
static int oa_tc6_process_rx_chunk(struct oa_tc6 *tc6,
				   struct net_pkt *pkt,
				   uint8_t *chunk_data,
				   uint32_t ftr)
{
	/*
	 * TODO:
	 * Move the existing per-chunk body from oa_tc6_read_chunks() here.
	 *
	 * Return values:
	 *   0        - one complete frame is ready
	 *   -EAGAIN  - this chunk was consumed, but the frame is not complete yet
	 *   < 0      - error
	 */
	return -EAGAIN;
}

int oa_tc6_read_chunks(struct oa_tc6 *tc6, struct net_pkt *pkt)
{
	const uint16_t buf_rx_size = CONFIG_NET_BUF_DATA_SIZE;
	struct net_buf *buf_rx = NULL;
	uint32_t buf_rx_used = 0;
	int ret = 0;

	uint8_t temp_rx_data[OA_TC6_XFER_CHUNK_BUDGET * 64];
	uint32_t ftrs[OA_TC6_XFER_CHUNK_BUDGET];
	uint8_t chunks_to_read;
	uint8_t i;

	/*
	 * Special case - append already received data (extracted from previous
	 * chunk) to new packet.
	 */
	if (tc6->concat_buf) {
		net_pkt_append_buffer(pkt, tc6->concat_buf);
		tc6->concat_buf = NULL;
	}

	while (true) {
		uint8_t start_idx = 0;
		uint8_t chunk_cnt = 0;
		uint8_t *rx_data_src = NULL;
		uint32_t *ftr_src = NULL;

		/* 1) Consume pending chunks first if any are available. */
		if (tc6->pending_idx < tc6->pending_cnt) {
			start_idx = tc6->pending_idx;
			chunk_cnt = tc6->pending_cnt;
			rx_data_src = tc6->pending_rx_data;
			ftr_src = tc6->pending_ftrs;
		} else {
			/* Clear the pending state before fetching a new burst. */
			tc6->pending_idx = 0;
			tc6->pending_cnt = 0;

			chunks_to_read = MIN((uint8_t)tc6->rca,
					     (uint8_t)OA_TC6_XFER_CHUNK_BUDGET);

			if (chunks_to_read == 0) {
				break;
			}

			ret = oa_tc6_chunk_spi_rx_transfer_burst(tc6, temp_rx_data, ftrs,
							      chunks_to_read);
			if (ret < 0) {
				LOG_ERR("OA RX burst: transmission error: %d!", ret);
				goto unref_buf;
			}

			start_idx = 0;
			chunk_cnt = chunks_to_read;
			rx_data_src = temp_rx_data;
			ftr_src = ftrs;
		}

		for (i = start_idx; i < chunk_cnt; i++) {
			uint32_t ftr = ftr_src[i];
			uint8_t *chunk_ptr = rx_data_src + (i * tc6->cps);
			uint8_t sbo, ebo;
			uint8_t copy_offset = 0;
			uint8_t copy_len = tc6->cps;
			bool sv, ev;

			if (!buf_rx) {
				buf_rx = net_pkt_get_frag(pkt, buf_rx_size,
							  OA_TC6_BUF_ALLOC_TIMEOUT);
				if (!buf_rx) {
					LOG_ERR("OA RX: Can't allocate RX buffer for data!");
					return -ENOMEM;
				}
			}

			ret = -EIO;
			if (oa_tc6_get_parity(ftr)) {
				LOG_ERR("OA RX: Footer parity error!");
				goto unref_buf;
			}

			if (!FIELD_GET(OA_DATA_FTR_SYNC, ftr)) {
				LOG_ERR("OA RX: Configuration not SYNC'ed!");
				goto unref_buf;
			}

			if (!FIELD_GET(OA_DATA_FTR_DV, ftr)) {
				LOG_DBG("OA RX: Data chunk not valid, skip!");
				goto unref_buf;
			}

			sv = FIELD_GET(OA_DATA_FTR_SV, ftr);
			ev = FIELD_GET(OA_DATA_FTR_EV, ftr);
			sbo = FIELD_GET(OA_DATA_FTR_SWO, ftr) * sizeof(uint32_t);
			ebo = FIELD_GET(OA_DATA_FTR_EBO, ftr) + 1;

			if (sv) {
				/*
				* Apply the start offset unless two frames are concatenated
				* within the same chunk.
				*/
				if (!(ev && (ebo <= sbo))) {
					copy_offset = sbo;
					copy_len = tc6->cps - copy_offset;
				}
			}

			if (ev) {
				/*
				* Drop the frame if the MAC requested frame discard.
				*/
				if (FIELD_GET(OA_DATA_FTR_FD, ftr)) {
					ret = -EIO;
					goto unref_buf;
				}

				/*
				* Handle the case where the current chunk contains both the
				* end of the previous frame and the start of the next frame.
				*/
				if (sv && (ebo <= sbo)) {
					copy_offset = 0;
					copy_len = ebo;
				} else {
					copy_len = ebo - copy_offset;
				}
			}

			if (copy_len > 0) {
				/*
				* If the current fragment does not have enough room,
				* append it and allocate a new fragment.
				*/
				if ((buf_rx_size - buf_rx_used) < copy_len) {
					buf_rx->len = buf_rx_used;
					net_pkt_append_buffer(pkt, buf_rx);
					buf_rx = NULL;
					buf_rx_used = 0;

					buf_rx = net_pkt_get_frag(pkt, buf_rx_size,
								  OA_TC6_BUF_ALLOC_TIMEOUT);
					if (!buf_rx) {
						LOG_ERR("OA RX: Can't allocate RX buffer for data!");
						return -ENOMEM;
					}
				}

				memcpy(buf_rx->data + buf_rx_used,
				       chunk_ptr + copy_offset,
				       copy_len);
				buf_rx_used += copy_len;
			}

			if (ev) {
				/*
				* If the chunk contains both the end of the current frame and
				* the head of the next frame, store the next frame head in concat_buf.
				*/
				if (sv && (ebo <= sbo)) {
					uint8_t next_len = tc6->cps - sbo;

					tc6->concat_buf = net_pkt_get_frag(pkt, next_len,
									   OA_TC6_BUF_ALLOC_TIMEOUT);
					if (!tc6->concat_buf) {
						LOG_ERR("OA RX: Can't allocate concat buffer!");
						ret = -ENOMEM;
						goto unref_buf;
					}

					memcpy(tc6->concat_buf->data, chunk_ptr + sbo, next_len);
					tc6->concat_buf->len = next_len;
				}

				buf_rx->len = buf_rx_used;
				net_pkt_append_buffer(pkt, buf_rx);
				buf_rx = NULL;

				/*
				* Store the remaining chunks in the burst as pending data.
				*/
				if ((i + 1) < chunk_cnt) {
					uint8_t remain = chunk_cnt - (i + 1);

					memcpy(tc6->pending_rx_data,
					       rx_data_src + ((i + 1) * tc6->cps),
					       remain * tc6->cps);

					memcpy(tc6->pending_ftrs,
					       &ftr_src[i + 1],
					       remain * sizeof(uint32_t));

					tc6->pending_idx = 0;
					tc6->pending_cnt = remain;
				} else {
					tc6->pending_idx = 0;
					tc6->pending_cnt = 0;
				}

				return 0;
			}
		}

		/*
		* Clear the pending state after all pending chunks are consumed.
		*/
		if (rx_data_src == tc6->pending_rx_data) {
			tc6->pending_idx = 0;
			tc6->pending_cnt = 0;
		}

		/*
		* Reaching this point means:
		* - no end-of-frame marker was found in the current burst
		* - the frame is still in progress
		* Continue reading the next burst in the loop.
		*/
	}

	return 0;

unref_buf:
	if (buf_rx) {
		net_buf_unref(buf_rx);
	}
	return ret;
}

/*
 * Build an RX OA-TC6 transfer plan.
 *
 * RX planning is based on RCA. RCA is updated from OA-TC6 data footers and
 * represents the number of receive chunks currently available in the MAC-PHY.
 *
 * Unlike TX planning, RX cannot use packet length because the host does not
 * know the incoming frame size before harvesting chunks. Therefore, RX uses
 * RCA as the demand value and the common OA-TC6 transaction budget as the
 * upper bound.
 */
static int oa_tc6_build_rx_plan(struct oa_tc6 *tc6,
				struct oa_tc6_xfer_plan *plan)
{
	if ((tc6 == NULL) || (plan == NULL)) {
		return -EINVAL;
	}

	memset(plan, 0, sizeof(*plan));

	if (tc6->rca == 0U) {
		return 0;
	}

	plan->rx_chunks = MIN((uint8_t)tc6->rca,
			      (uint8_t)OA_TC6_XFER_CHUNK_BUDGET);
	plan->total_chunks = plan->rx_chunks;

	return 0;
}


static int oa_tc6_spi_data_xfer(struct oa_tc6 *tc6,
				const uint8_t *tx_buf,
				uint8_t *rx_buf,
				size_t len,
				uint16_t tx_chunks)
{
	int ret;

	if ((tc6 == NULL) || (tx_buf == NULL) || (rx_buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	ret = spi_stream_runtime_set_tx_chunks(tc6->spi->bus, tx_chunks);
	if (ret < 0) {
		return ret;
	}

	ret = spi_transceive_stream_dt(tc6->spi, tx_buf, rx_buf, len);

	spi_stream_runtime_clear(tc6->spi->bus);

	return ret;
}

// for debugging
static void oa_tc6_log_rx_stream(const uint8_t *rx_buf, size_t len)
{
	size_t i;
	size_t dump_len = MIN(len, 128U); /* first stage: only first 128B */

	if (rx_buf == NULL || len == 0U) {
		LOG_ERR("OA RX stream: empty");
		return;
	}

	LOG_ERR("OA RX stream dump: len=%u dump_len=%u",
		(unsigned int)len, (unsigned int)dump_len);

	for (i = 0; i < dump_len; i += 16U) {
		char line[80];
		int pos = 0;
		size_t j;
		size_t line_len = MIN((size_t)16U, dump_len - i);

		pos += snprintk(line + pos, sizeof(line) - pos, "%04u:", (unsigned int)i);

		for (j = 0; j < line_len; j++) {
			pos += snprintk(line + pos, sizeof(line) - pos,
					" %02x", rx_buf[i + j]);
		}

		LOG_ERR("%s", line);
	}
}

/*
 * Build a TX-first OA-TC6 transfer plan.
 *
 * TX planning is based on the remaining packet payload length. The number
 * of TX chunks is capped by the common OA-TC6 transaction budget.
 *
 * Credit-aware scheduling will be added later when the TX/RX scheduler is
 * introduced.
 */
static int oa_tc6_build_xfer_plan(struct oa_tc6 *tc6,
				  size_t remaining,
				  struct oa_tc6_xfer_plan *plan)
{
	uint8_t needed_chunks;

	if ((tc6 == NULL) || (plan == NULL)) {
		return -EINVAL;
	}

	memset(plan, 0, sizeof(*plan));

	if (remaining == 0U) {
		return 0;
	}

	needed_chunks = DIV_ROUND_UP(remaining, tc6->cps);

	plan->tx_chunks = MIN(needed_chunks, OA_TC6_XFER_CHUNK_BUDGET);
	plan->total_chunks = plan->tx_chunks;

	return 0;
}

/*
 * Run the common OA-TC6 data transfer loop.
 *
 * This initial implementation is TX-first and credit-based. It uses the
 * current TX credit value to decide how many chunks can be transmitted
 * in one transaction, refreshes status when credit is unavailable, and
 * repeats until the full packet has been consumed.
 *
 * RX chunk harvesting will be added later without changing the outer
 * execution flow.
 */
static int oa_tc6_run_data_xfer(struct oa_tc6 *tc6, struct net_pkt *pkt)
{
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t remaining;
	uint8_t chunk_idx = 0U;
	int ret;

	if ((tc6 == NULL) || (pkt == NULL)) {
		return -EINVAL;
	}

	remaining = net_pkt_get_len(pkt);
	if (remaining == 0U) {
		return -ENODATA;
	}

	/*
	 * Reuse the persistent OA-TC6 work buffers.
	 * TX and RX buffers are both prepared here because the transfer path
	 * must now go through the stream-capable SPI path instead of the old
	 * TX-only burst helper.
	 */
	tx_buf = tc6->spi_data_tx_buf;
	rx_buf = tc6->spi_data_rx_buf;

	while (remaining > 0U) {
		struct oa_tc6_xfer_plan plan;
		size_t xfer_len;
		uint8_t i;

		ret = oa_tc6_build_xfer_plan(tc6, remaining, &plan);
		if (ret < 0) {
			return ret;
		}

		if (plan.tx_chunks == 0U) {
			LOG_ERR("OA TX: invalid xfer plan (remaining=%u)",
				(unsigned int)remaining);
			return -EIO;
		}

		xfer_len = (size_t)plan.total_chunks * OA_TC6_TX_CHUNK_SIZE;

		memset(tx_buf, 0, xfer_len);
		memset(rx_buf, 0, xfer_len);

		for (i = 0U; i < plan.tx_chunks; i++) {
			uint8_t *chunk_ptr = &tx_buf[i * OA_TC6_TX_CHUNK_SIZE];
			uint8_t *payload_ptr = chunk_ptr + OA_TC6_HDR_SIZE;
			size_t copy_len = MIN(remaining, (size_t)tc6->cps);
			uint32_t hdr = 0U;

			/*
			 * Keep the normal OA-TC6 data chunk format.
			 *
			 * At this stage, TX chunk count is payload-driven, but the
			 * actual transfer must go through the stream-capable SPI
			 * path so that RX service can happen in spi_dw.
			 *
			 * NORX is intentionally not set here, because the balanced
			 * TX/RX design now depends on the lower SPI path being able
			 * to receive and drain return data as needed.
			 */
			hdr |= FIELD_PREP(OA_DATA_HDR_DNC, 1);
			hdr |= FIELD_PREP(OA_DATA_HDR_DV, 1);

			if (chunk_idx == 0U) {
				hdr |= FIELD_PREP(OA_DATA_HDR_SV, 1);
			}

			if (remaining <= tc6->cps) {
				hdr |= FIELD_PREP(OA_DATA_HDR_EV, 1);
				hdr |= FIELD_PREP(OA_DATA_HDR_EBO, copy_len - 1U);
			}

			hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));
			sys_put_be32(hdr, chunk_ptr);

			ret = net_pkt_read(pkt, payload_ptr, copy_len);
			if (ret < 0) {
				LOG_ERR("OA TX: failed to read packet data (%d)", ret);
				return ret;
			}

			remaining -= copy_len;
			chunk_idx++;
		}

		/*
		 * Run the transfer through the full-duplex stream path.
		 *
		 * The current step focuses on payload-driven TX planning while
		 * letting spi_dw_xfer_polling_stream8() evolve into the engine
		 * that balances TX progress and RX service.
		 */
		ret = oa_tc6_spi_data_xfer(tc6, tx_buf, rx_buf, xfer_len, plan.tx_chunks);
		if (ret < 0) {
			LOG_ERR("OA TX: SPI transfer failed (%d)", ret);
			return ret;
		}

		/*
		 * Status/footer consumption is intentionally deferred for now.
		 *
		 * The immediate goal is to validate that:
		 *  1) the TX plan is driven by payload size, and
		 *  2) the lower SPI stream path can keep RX alive without
		 *     falling back to the removed TX-only burst helper.
		 *
		 * Credit/status synchronization can be reintroduced after the
		 * stream path behavior is measured under TX-only, RX-only,
		 * and mixed traffic.
		 */
	}

	return 0;
}

int oa_tc6_run_tx(struct oa_tc6 *tc6, struct net_pkt *pkt)
{
	if ((tc6 == NULL) || (pkt == NULL)) {
		return -EINVAL;
	}

	if (net_pkt_get_len(pkt) == 0U) {
		return -ENODATA;
	}

#if defined(CONFIG_ETH_LAN865X_OA_TC6_CREDIT_BASED_XFER)
	int ret = oa_tc6_run_data_xfer(tc6, pkt);
	return ret;
#else
	return oa_tc6_send_chunks(tc6, pkt);
#endif
}
