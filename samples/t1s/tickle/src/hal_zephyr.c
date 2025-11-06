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

	struct ethhdr *eth = (struct ethhdr *)tmpbuf;

	memset(eth->h_dest, 0xFF, ETH_ALEN); // Broadcast
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(0x1337);

	memcpy(tmpbuf + sizeof(struct ethhdr), buf, len);

	int ret = send_packet(node->hal.spi, tmpbuf, len + sizeof(struct ethhdr));
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

	memcpy(buf, tmpbuf + sizeof(struct ethhdr), length - sizeof(struct ethhdr));

	*ip = 0;
	*port = 0;

	return length - sizeof(struct ethhdr);
}
