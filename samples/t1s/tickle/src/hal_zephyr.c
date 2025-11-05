#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/netinet/in.h>
#include <zephyr/posix/sys/socket.h>

#include <tickle/config.h>
#include <tickle/hal.h>
#include <tickle/hal_zephyr.h>
#include <tickle/tickle.h>

#include "../../src/lan8651.h"
#include "../../src/eth.h"

#include "consts.h"
#include "log.h"

#define SEC_NS 1000000000ULL

#define UNUSED(x) (void)(x)

extern struct spi_dt_spec spi_dev;

struct _tt_Config _tt_CONFIG = {
	.addr = _tt_NODE_ADDRESS,
	.port = _tt_NODE_PORT,
	.broadcast = _tt_NODE_BROADCAST,
};

static uint16_t csum16(const void *data, size_t len) {
    uint32_t sum = 0;
    const uint16_t *p = (const uint16_t *)data;
    while (len > 1) { sum += *p++; len -= 2; }
    if (len == 1) {
        uint16_t last = 0;
        *(uint8_t*)(&last) = *(const uint8_t*)p;
        sum += last;
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

extern uint8_t my_mac_addr[ETH_ALEN];
extern uint8_t target_mac_addr[ETH_ALEN];
extern uint8_t my_ip_addr[IP_LEN];
extern uint8_t target_ip_addr[IP_LEN];
extern uint8_t tickle_node_id;

uint64_t tt_get_ns()
{
	struct timespec ts;
	sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);

	return ((uint64_t)ts.tv_sec * SEC_NS) + ts.tv_nsec;
}

int32_t tt_get_node_id()
{
	/* TODO: use zephyr config */
	return tickle_node_id;
}

int32_t tt_bind(struct tt_Node *node)
{
	node->hal.spi = &spi_dev;
	return 0;
}

void tt_close(struct tt_Node *node)
{
}

int32_t tt_send(struct tt_Node *node, const void *buf, size_t len)
{
	uint8_t tmpbuf[4096] = {0};

	printk("send: %u\n", len);
	/* Debug: dump buf in 8-byte chunks */
	const uint8_t *bytes = (const uint8_t *)buf;
	for (size_t i = 0; i < len; i += 8) {
		size_t chunk_len = (len - i) < 8 ? (len - i) : 8;
		printk("buf[%4u..%4u]:", (unsigned)i, (unsigned)(i + chunk_len - 1));
		for (size_t j = 0; j < chunk_len; ++j) {
			printk(" %02x", bytes[i + j]);
		}
		printk("\n");
	}

	struct ethhdr *eth = (struct ethhdr *)tmpbuf;
	// struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
	// struct udphdr *udp = (struct udphdr *)(ipv4 + 1);

	memset(eth->h_dest, 0xFF, ETH_ALEN); // Broadcast
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(0x1337);
	// ipv4->version = 4;
	// ipv4->ihl = 5;
	// ipv4->tos = 0;
	// ipv4->len = sys_cpu_to_be16(len + sizeof(struct ethhdr));
	// ipv4->id = 0;
	// ipv4->frag_off = 0;
	// ipv4->ttl = 64;
	// ipv4->proto = IPPROTO_UDP;
	// // memcpy(&ipv4->src, sender_ip_addr, IP_LEN);
	// // memcpy(&ipv4->dst, receiver_ip_addr, IP_LEN);
	// memcpy(&ipv4->src, receiver_ip_addr, IP_LEN);
	// memcpy(&ipv4->dst, sender_ip_addr, IP_LEN);
	// ipv4->chksum = 0;
	// udp->source_port = sys_cpu_to_be16(1234);
	// udp->dest_port = sys_cpu_to_be16(5678);
	// udp->length = sys_cpu_to_be16(len + sizeof(struct udphdr));
	// udp->checksum = 0;

	memcpy(tmpbuf + sizeof(struct ethhdr), buf, len);
	// memcpy(tmpbuf + sizeof(struct ethhdr) + sizeof(struct ipv4hdr) + sizeof(struct udphdr), buf, len);

	printk("send 1\n");
	int ret = send_packet(node->hal.spi, tmpbuf, len + sizeof(struct ethhdr));
	printk("send 2\n");
	if (ret != 0) {
		return -1;
	}
	return 0;
}

int32_t tt_receive(struct tt_Node *node, void *buf, size_t len, uint32_t *ip, uint16_t *port)
{
	uint16_t length;
	uint8_t tmpbuf[4096] = {0};
	int ret = receive_packet(node->hal.spi, tmpbuf, &length);

	if (ret != 0) {
		printk("Failed to receive packet\n");
		return -1;
	}
	if (length > len) {
		printk("Not enough buffer\n");
		return -1;
	}
	if (length == 0) {
		/* No data received */
		return -1;
	}

	printk("received %u\n", length);
	memcpy(buf, tmpbuf + sizeof(struct ethhdr), length - sizeof(struct ethhdr));

	const uint8_t *bytes = (const uint8_t *)tmpbuf;
	for (size_t i = 0; i < length; i += 8) {
		size_t chunk_len = (length - i) < 8 ? (length - i) : 8;
		printk("buf[%4u..%4u]:", (unsigned)i, (unsigned)(i + chunk_len - 1));
		for (size_t j = 0; j < chunk_len; ++j) {
			printk(" %02x", bytes[i + j]);
		}
		printk("\n");
	}

	*ip = 0;
	*port = 0;

	return length - sizeof(struct ethhdr);
}
