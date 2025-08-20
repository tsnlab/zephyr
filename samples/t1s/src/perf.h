#ifndef PERF_H
#define PERF_H

#include <stdint.h>
#include <zephyr/drivers/spi.h>

#include "eth.h"

struct perf_header {
	uint32_t id;
	uint8_t op;
} __attribute__((packed));

struct perf_startreq_header {
	struct perf_header header;
	uint32_t duration;
	uint32_t warmup;
} __attribute__((packed));

struct perf_latency_header {
	uint32_t id;
	uint8_t op;
	uint32_t tv_sec;
	uint32_t tv_usec;
} __attribute__((packed));

enum perf_throughput_op {
	PERF_REQ_START = 0x00,
	PERF_REQ_END = 0x01,
	PERF_RES_START = 0x20,
	PERF_RES_END = 0x21,
	PERF_DATA = 0x30,
	PERF_REQ_RESULT = 0x40,
	PERF_RES_RESULT = 0x41,
};

enum perf_latency_op {
	PERF_PING = 0,
	PERF_PONG = 1,
	PERF_TX = 2,
	PERF_SYNC = 3,
};

#define PERF_ETHERTYPE   0x1337
#define MAX_PERF_PAYLOAD 1500

int send_perf_req_packet(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
			 const uint8_t target_mac_addr[ETH_ALEN], uint32_t duration,
			 uint32_t warmup);
uint16_t make_perf_data_packet(const uint8_t my_mac_addr[ETH_ALEN],
			       const uint8_t target_mac_addr[ETH_ALEN], uint8_t *packet,
			       uint32_t len);
int send_perf_data_packet(const struct spi_dt_spec *spi, const uint8_t *packet, uint16_t length);

int recv_latency_req(const struct spi_dt_spec *spi, uint8_t source_mac_addr[ETH_ALEN],
		     uint32_t *id);
uint16_t make_latency_res(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
			  const uint8_t target_mac_addr[ETH_ALEN], uint32_t id);
int send_latency_res(const struct spi_dt_spec *spi, const uint8_t *packet, uint16_t length);

#endif /* PERF_H */
