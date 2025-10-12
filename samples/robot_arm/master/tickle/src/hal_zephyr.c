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
#include <tickle/tickle.h>

#include "consts.h"
#include "log.h"

#define SEC_NS 1000000000ULL

#define UNUSED(x) (void)(x)

struct _tt_Config _tt_CONFIG = {
    .addr = _tt_NODE_ADDRESS,
    .port = _tt_NODE_PORT,
    .broadcast = _tt_NODE_BROADCAST,
};

uint64_t tt_get_ns() {
    struct timespec ts;
    sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);

    return ((uint64_t)ts.tv_sec * SEC_NS) + ts.tv_nsec;
}

int32_t tt_get_node_id() {
    // Get unique node ID in the network using IP address x.x.x.id
    uint32_t broadcast_ip = inet_addr(_tt_CONFIG.broadcast);
    struct net_if* iface;

    uint8_t node_id = 0;

    iface = net_if_get_default();  /* TODO: Use net_if_foreach() */
    if (iface == NULL) {
        printk("No default network interface found\n");  /* TODO: Use LOG_ERR */
        return -1;
    }
    if (iface->config.ip.ipv4 == NULL) {
        net_if_get_by_index(1);
    }
    if (iface->config.ip.ipv4 == NULL) {
        net_if_get_by_index(2);
    }
    if (iface == NULL) {
        printk("No network interface found\n");
        return -1;
    }

    uint8_t* _addr = iface->config.ip.ipv4->unicast[0].ipv4.address.in_addr.s4_addr;
    printk("addr: %u.%u.%u.%u\n", _addr[0], _addr[1], _addr[2], _addr[3]);
    uint8_t* _linkaddr = iface->if_dev->link_addr.addr;
    printk("linkaddr: %02x:%02x:%02x:%02x:%02x:%02x\n", _linkaddr[0], _linkaddr[1], _linkaddr[2], _linkaddr[3], _linkaddr[4], _linkaddr[5]);

    uint32_t addr = iface->config.ip.ipv4->unicast[0].ipv4.address.in_addr.s_addr;
    uint32_t netmask = iface->config.ip.ipv4->unicast[0].netmask.s_addr;
    if (node_id == 0) {
        node_id = addr >> 24;
    } else if ((addr & netmask) == (broadcast_ip & netmask)) {
        node_id = (addr & ~netmask) >> BITS_IN_3BYTES;
    }

    return node_id;
}

int32_t tt_bind(struct tt_Node* node) {
    node->hal.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (node->hal.sock < 0) {
        TT_LOG_ERROR("Cannot create UDP socket");
        return tt_CANNOT_CREATE_SOCK;
    }

    int optval = 1;
    if (setsockopt(node->hal.sock, SOL_SOCKET, SO_REUSEADDR, (const void*)&optval, sizeof(int)) < 0) {
        TT_LOG_ERROR("Cannot set socket reuseaddr");
        return tt_CANNOT_SET_REUSEADDR;
    }

    // optval = 1;
    // if (setsockopt(node->hal.sock, SOL_SOCKET, SO_BROADCAST, (const void*)&optval, sizeof(int)) < 0) {
    //     TT_LOG_ERROR("Cannot set socket broadcast");
    //     return tt_CANNOT_SET_BROADCAST;
    // }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = TIMEOUT_IN_MICROSECONDS;

    if (setsockopt(node->hal.sock, SOL_SOCKET, SO_RCVTIMEO, (const void*)&timeout, sizeof(struct timeval)) < 0) {
        // TT_LOG_ERROR("Cannot set socket receive timeout");
        printk("Cannot set socket receive timeout");
        return tt_CANNOT_SET_TIMEOUT;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(_tt_CONFIG.addr);
    addr.sin_port = htons(_tt_CONFIG.port);

    if (bind(node->hal.sock, (struct sockaddr*)&addr, sizeof(struct sockaddr_in)) < 0) {
        TT_LOG_ERROR("Cannot binding socket: %s:%d", _tt_CONFIG.addr, _tt_CONFIG.port);
        TT_LOG_ERROR("Cannot binding socket\n");
        return tt_CANNOT_BIND_SOCKET;
    }

    return 0;
}

void tt_close(struct tt_Node* node) {
    if (close(node->hal.sock) == -1) {
        TT_LOG_ERROR("Cannot close socket: %s", strerror(errno));
    }
}

int32_t tt_send(struct tt_Node* node, const void* buf, size_t len) {
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(_tt_CONFIG.broadcast);
    addr.sin_port = htons(_tt_CONFIG.port);

    return (int32_t)sendto(node->hal.sock, buf, len, 0, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
}

int32_t tt_receive(struct tt_Node* node, void* buf, size_t len, uint32_t* ip, uint16_t* port) {
    struct sockaddr_in addr;
    int addr_len = sizeof(struct sockaddr_in);

    printk("receive\n");
    int32_t ret = (int32_t)recvfrom(node->hal.sock, buf, len, 0, (struct sockaddr*)&addr, &addr_len);
    printk("receive end\n");

    *ip = ntohl(addr.sin_addr.s_addr);
    *port = ntohs(addr.sin_port);

    return ret;
}
