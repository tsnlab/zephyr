#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "perf.h"
#include "udp.h"
#include "lan8651.h"

void make_udp_perf_req_packet(uint8_t *udp_payload, uint32_t duration_ms, uint32_t warmup_ms)
{
	struct perf_startreq_header *startreq = (struct perf_startreq_header *)(udp_payload);

	startreq->header.id = sys_cpu_to_be32(0xdeadbeef);
	startreq->header.op = PERF_REQ_START;

	startreq->duration = sys_cpu_to_be32(duration_ms);
	startreq->warmup = sys_cpu_to_be32(warmup_ms);
}

void make_udp_perf_data_packet(uint8_t *udp_payload)
{
    struct perf_header *perf = (struct perf_header *)(udp_payload);

    perf->id = sys_cpu_to_be32(0xbeef);
    perf->op = PERF_DATA;
}

int send_udp_perf_req_packet(const struct spi_dt_spec *spi, const uint8_t *udp_packet, uint16_t packet_length)
{
	return send_packet(spi, udp_packet, packet_length);
}

int send_udp_perf_data_packet(const struct spi_dt_spec *spi, const uint8_t *udp_packet, uint16_t packet_length)
{
	if (packet_length > MAX_PERF_PAYLOAD) {
		printk("Length is too long\n");
		return -1;
	}
	uint8_t txbuffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};
	union data_header data_transfer_header;

	data_transfer_header.data_frame_head = 0;
	uint32_t be_header;

	uint32_t chunk_count = packet_length / MAX_PAYLOAD_BYTE;
	if (packet_length % MAX_PAYLOAD_BYTE) {
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
			chunk_size = packet_length % MAX_PAYLOAD_BYTE;
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

		memcpy(&txbuffer[HEADER_SIZE], udp_packet + i * MAX_PAYLOAD_BYTE, MAX_PAYLOAD_BYTE);

		struct spi_buf tx_buf = {.buf = txbuffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
		const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
		struct spi_buf rx_buf = {.buf = rxbuffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
		const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
		spi_transceive_dt(spi, &tx_buf_set, &rx_buf_set);
	}

	return 0;
}

int recv_udp_perf_req(const struct spi_dt_spec *spi, uint8_t source_mac_addr[ETH_ALEN], 
    uint8_t source_ip_addr[IP_LEN], uint16_t* source_port, uint32_t *duration)
{
	uint8_t packet[MAX_PACKET_SIZE] = {0,};
	uint16_t length = 0;

	while (length == 0) {
		if (receive_packet(spi, packet, &length) < 0) {
			//printk("Failed to receive Perf request\n");
			return -1;
		}
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	if (eth->h_proto != sys_cpu_to_be16(ETH_P_IP)) {
		//printk("Non-IP packet received %d\n", eth->h_proto);
		return -1;
	}
    memcpy(source_mac_addr, eth->h_source, ETH_ALEN);

    struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
    if (ipv4->proto != IPPROTO_UDP) {
        //printk("Non-UDP packet received %d\n", ipv4->proto);
        return -1;
    }
    memcpy(source_ip_addr, ipv4->src, IP_LEN);

    struct udphdr *udp = (struct udphdr *)(ipv4 + 1);
    if (udp->dest_port != sys_cpu_to_be16(PERF_UDP_PORT)) {
        //printk("Non-Perf UDP packet received %d\n", udp->dest_port);
        return -1;
    }
    *source_port = sys_be16_to_cpu(udp->source_port);

	struct perf_startreq_header *perf = (struct perf_startreq_header *)(eth + 1);
	if (perf->header.op != PERF_REQ_START) {
		//printk("Non-Req packet received %d\n", perf->header.op);
		return -1;
	}

	*duration = sys_be32_to_cpu(perf->duration);

	return length;
}

int recv_udp_perf_data(const struct spi_dt_spec *spi, uint16_t req_source_port, uint32_t *id)
{
	uint8_t packet[MAX_PACKET_SIZE] = {0};
	uint16_t length = 0;

	while (length == 0) {
		if (receive_packet(spi, packet, &length) < 0) {
			//printk("Failed to receive Perf data\n");
			return -1;
		}
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	if (eth->h_proto != sys_cpu_to_be16(ETH_P_IP)) {
		//printk("Non-IP packet received %d\n", eth->h_proto);
		return -1;
	}

    struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
    if (ipv4->proto != IPPROTO_UDP) {
        //printk("Non-UDP packet received %d\n", ipv4->proto);
        return -1;
    }

    struct udphdr *udp = (struct udphdr *)(ipv4 + 1);
    if (udp->dest_port != sys_cpu_to_be16(PERF_UDP_PORT)) {
        //printk("Non-Perf UDP packet received %d\n", udp->dest_port);
        return -1;
    }

    if (udp->source_port != sys_cpu_to_be16(req_source_port)) {
        printk("Source port mismatch %d != %d\n", udp->source_port, req_source_port);
        return -1;
    }

	struct perf_header *perf = (struct perf_header *)(udp + 1);
	if (perf->op != PERF_DATA) {
		printk("Non-Data packet received %d\n", perf->op);
		return -1;
	}

	*id = sys_be32_to_cpu(perf->id);

	return length;
}
