#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "perf.h"
#include "eth.h"
#include "lan8651.h"

static uint16_t make_perf_req_packet(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
				     const uint8_t target_mac_addr[ETH_ALEN], uint32_t duration,
				     uint32_t warmup)
{
	struct ethhdr *eth = (struct ethhdr *)packet;
	struct perf_startreq_header *startreq = (struct perf_startreq_header *)(eth + 1);

	memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(PERF_ETHERTYPE);

	startreq->header.id = sys_cpu_to_be32(0xdeadbeef);
	startreq->header.op = PERF_REQ_START;

	startreq->duration = sys_cpu_to_be32(duration);
	startreq->warmup = sys_cpu_to_be32(warmup);

	return sizeof(struct ethhdr) + sizeof(struct perf_startreq_header);
}

int send_perf_req_packet(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
			 const uint8_t target_mac_addr[ETH_ALEN], uint32_t duration,
			 uint32_t warmup)
{
	uint8_t packet[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint16_t length =
		make_perf_req_packet(packet, my_mac_addr, target_mac_addr, duration, warmup);

	uint8_t txbuffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};
	union data_header data_transfer_header;

	data_transfer_header.data_frame_head = 0;
	uint32_t be_header;

	data_transfer_header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	data_transfer_header.tx_header_bits.norx = NORX_NO_RECEIVE;
	data_transfer_header.tx_header_bits.dv = DV_DATA_VALID;
	data_transfer_header.tx_header_bits.sv = SV_START_VALID;
	data_transfer_header.tx_header_bits.ev = EV_END_VALID;
	data_transfer_header.tx_header_bits.ebo = length - 1;
	data_transfer_header.tx_header_bits.p = get_parity(data_transfer_header.data_frame_head);

	be_header = sys_cpu_to_be32(data_transfer_header.data_frame_head);
	memcpy(txbuffer, &be_header, HEADER_SIZE);

	memcpy(&txbuffer[HEADER_SIZE], packet, length);

	struct spi_buf tx_buf = {.buf = txbuffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
	const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = rxbuffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
	const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
	spi_transceive_dt(spi, &tx_buf_set, &rx_buf_set);

	return 0;
}

uint16_t make_perf_data_packet(const uint8_t my_mac_addr[ETH_ALEN],
			       const uint8_t target_mac_addr[ETH_ALEN], uint8_t *packet,
			       uint32_t len)
{
	if (len < sizeof(struct ethhdr) + sizeof(struct perf_header)) {
		printk("Length is too short: %u\n", len);
		return 0;
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct perf_header *perf = (struct perf_header *)(eth + 1);

	memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(PERF_ETHERTYPE);

	perf->id = sys_cpu_to_be32(0x0);
	perf->op = PERF_DATA;

	return len;
}

int send_perf_data_packet(const struct spi_dt_spec *spi, const uint8_t *packet, uint16_t length)
{
	if (length > MAX_PERF_PAYLOAD) {
		printk("Length is too long\n");
		return -1;
	}
	uint8_t txbuffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};
	union data_header data_transfer_header;

	data_transfer_header.data_frame_head = 0;
	uint32_t be_header;

	uint32_t chunk_count = length / MAX_PAYLOAD_BYTE;
	if (length % MAX_PAYLOAD_BYTE) {
		chunk_count++;
	}

	uint32_t chunk_size = MAX_PAYLOAD_BYTE;
	data_transfer_header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	data_transfer_header.tx_header_bits.norx = NORX_NO_RECEIVE;
	data_transfer_header.tx_header_bits.dv = DV_DATA_VALID;
	for (uint32_t i = 0; i < chunk_count; i++) {
		data_transfer_header.tx_header_bits.sv = SV_START_INVALID;
		data_transfer_header.tx_header_bits.ev = EV_END_INVALID;
		data_transfer_header.tx_header_bits.ebo = 0;

		if (i == 0) { // First chunk
			data_transfer_header.tx_header_bits.sv = SV_START_VALID;
		}
		if (i == chunk_count - 1) { // Last chunk
			chunk_size = length % MAX_PAYLOAD_BYTE;
			if (chunk_size == 0) {
				chunk_size = MAX_PAYLOAD_BYTE;
			}
			data_transfer_header.tx_header_bits.ev = EV_END_VALID;
			data_transfer_header.tx_header_bits.ebo = chunk_size - 1;
		}

		data_transfer_header.tx_header_bits.p =
			get_parity(data_transfer_header.data_frame_head);

		be_header = sys_cpu_to_be32(data_transfer_header.data_frame_head);
		memcpy(txbuffer, &be_header, HEADER_SIZE);

		memcpy(&txbuffer[HEADER_SIZE], packet + i * MAX_PAYLOAD_BYTE, MAX_PAYLOAD_BYTE);

		struct spi_buf tx_buf = {.buf = txbuffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
		const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
		struct spi_buf rx_buf = {.buf = rxbuffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
		const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
		spi_transceive_dt(spi, &tx_buf_set, &rx_buf_set);
	}

	return 0;
}

int recv_latency_req(const struct spi_dt_spec *spi, uint8_t source_mac_addr[ETH_ALEN], uint32_t *id)
{
	uint8_t txbuffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};

	union data_header data_transfer_header;

	data_transfer_header.data_frame_head = 0;
	uint32_t be_header;
	data_transfer_header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	data_transfer_header.tx_header_bits.norx = NORX_RECEIVE;
	data_transfer_header.tx_header_bits.dv = DV_DATA_INVALID;
	data_transfer_header.tx_header_bits.sv = SV_START_INVALID;
	data_transfer_header.tx_header_bits.ev = EV_END_INVALID;
	data_transfer_header.tx_header_bits.ebo = 0;

	data_transfer_header.tx_header_bits.p = get_parity(data_transfer_header.data_frame_head);

	be_header = sys_cpu_to_be32(data_transfer_header.data_frame_head);
	memcpy(txbuffer, &be_header, HEADER_SIZE);

	struct spi_buf tx_buf = {.buf = txbuffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
	const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = rxbuffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
	const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
	spi_transceive_dt(spi, &tx_buf_set, &rx_buf_set);

	union data_footer data_footer;
	data_footer.data_frame_foot = sys_be32_to_cpu(*(uint32_t *)(rxbuffer + MAX_PAYLOAD_BYTE));
	if (data_footer.rx_footer_bits.p != get_parity(data_footer.data_frame_foot)) {
		printk("Parity error while receiving packet\n");
		return -1;
	}

	if (data_footer.rx_footer_bits.dv != DV_DATA_VALID) { /* No data to be received */
		return 0;
	}

	memcpy(source_mac_addr, rxbuffer, ETH_ALEN);
	struct ethhdr *eth = (struct ethhdr *)rxbuffer;
	if (eth->h_proto != sys_cpu_to_be16(PERF_ETHERTYPE)) { /* Ignore Non-Perf packets */
		return 0;
	}

	struct perf_latency_header *perf = (struct perf_latency_header *)(eth + 1);
	*id = sys_be32_to_cpu(perf->id);
	printk("Received latency request from %02x:%02x:%02x:%02x:%02x:%02x, id: %u\n",
	       source_mac_addr[0], source_mac_addr[1], source_mac_addr[2], source_mac_addr[3],
	       source_mac_addr[4], source_mac_addr[5], *id);

	return 0;
}

uint16_t make_latency_res(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
			  const uint8_t target_mac_addr[ETH_ALEN], uint32_t id)
{
	struct ethhdr *eth = (struct ethhdr *)packet;
	struct perf_latency_header *perf = (struct perf_latency_header *)(eth + 1);

	memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(PERF_ETHERTYPE);

	perf->id = sys_cpu_to_be32(id);
	perf->op = PERF_RES_START;

	return sizeof(struct ethhdr) + sizeof(struct perf_latency_header);
}

int send_latency_res(const struct spi_dt_spec *spi, const uint8_t *packet, uint16_t length)
{
	uint8_t txbuffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};

	union data_header data_transfer_header;

	data_transfer_header.data_frame_head = 0;
	uint32_t be_header;
	data_transfer_header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	data_transfer_header.tx_header_bits.norx = NORX_NO_RECEIVE;
	data_transfer_header.tx_header_bits.dv = DV_DATA_VALID;
	data_transfer_header.tx_header_bits.sv = SV_START_VALID;
	data_transfer_header.tx_header_bits.ev = EV_END_VALID;
	data_transfer_header.tx_header_bits.ebo = 0;

	data_transfer_header.tx_header_bits.p = get_parity(data_transfer_header.data_frame_head);

	be_header = sys_cpu_to_be32(data_transfer_header.data_frame_head);
	memcpy(txbuffer, &be_header, HEADER_SIZE);

	memcpy(&txbuffer[HEADER_SIZE], packet, length);

	struct spi_buf tx_buf = {.buf = txbuffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
	const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = rxbuffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
	const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
	spi_transceive_dt(spi, &tx_buf_set, &rx_buf_set);

	return 0;
}
