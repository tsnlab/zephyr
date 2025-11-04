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

#include <lan8651/lan8651.h>

#include "consts.h"
#include "log.h"

#define SEC_NS 1000000000ULL

#define UNUSED(x) (void)(x)

struct _tt_Config _tt_CONFIG = {
	.addr = _tt_NODE_ADDRESS,
	.port = _tt_NODE_PORT,
	.broadcast = _tt_NODE_BROADCAST,
};

uint64_t tt_get_ns()
{
	struct timespec ts;
	sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);

	return ((uint64_t)ts.tv_sec * SEC_NS) + ts.tv_nsec;
}

int32_t tt_get_node_id()
{
	/* TODO: use zephyr config */
	return 1;
}

int32_t tt_bind(struct tt_Node *node)
{
	return 0;
}

void tt_close(struct tt_Node *node)
{
}

int32_t tt_send(struct tt_Node *node, const void *buf, size_t len)
{
	/* TODO: call send_block */
	return 0;
}

int32_t tt_receive(struct tt_Node *node, void *buf, size_t len, uint32_t *ip, uint16_t *port)
{
	int ret = receive_packet(&node->hal.spi, buf, len);

	if (ret != 0) {
		return -1;
	}

	*ip = 0;
	*port = 0;

	return ret;
}
