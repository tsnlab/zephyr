#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>

#define UDP_PORT         5000
#define BUF_SIZE         1600
#define TX_SIZE          1472

#define MY_IPV4_ADDR     "10.1.1.5"
#define MY_IPV4_MASK     "255.255.255.0"
#define PEER_IPV4_ADDR   "10.1.1.10"

static uint8_t tx_buf[BUF_SIZE];

static void print_mac(struct net_if *iface)
{
    const struct net_linkaddr *ll = net_if_get_link_addr(iface);

    if (!ll || ll->len < 6) {
        printk("MAC: invalid\n");
        return;
    }

    printk("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
           ll->addr[0], ll->addr[1], ll->addr[2],
           ll->addr[3], ll->addr[4], ll->addr[5]);
}

static int setup_ipv4_on_lan865x(void)
{
    const struct device *eth_dev;
    struct net_if *iface;
    struct net_if *found_iface = NULL;
    struct net_if_addr *ifaddr;
    struct in_addr addr;
    struct in_addr mask;
    char buf[NET_IPV4_ADDR_LEN];

    eth_dev = DEVICE_DT_GET(DT_NODELABEL(lan865x));
    if (!device_is_ready(eth_dev)) {
        printk("LAN865x device not ready\n");
        return -ENODEV;
    }

    iface = net_if_lookup_by_dev(eth_dev);
    if (iface == NULL) {
        printk("No iface found for LAN865x\n");
        return -ENODEV;
    }

    printk("LAN865x iface=%p\n", iface);
    print_mac(iface);

    if (!net_if_is_up(iface)) {
        net_if_up(iface);
    }

    while (!net_if_is_up(iface)) {
        printk("Waiting for interface up...\n");
        k_sleep(K_MSEC(200));
    }

    printk("Interface UP\n");

    if (net_addr_pton(AF_INET, MY_IPV4_ADDR, &addr) < 0) {
        printk("Invalid IPv4 address string\n");
        return -EINVAL;
    }

    if (net_addr_pton(AF_INET, MY_IPV4_MASK, &mask) < 0) {
        printk("Invalid IPv4 mask string\n");
        return -EINVAL;
    }

    ifaddr = net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    if (ifaddr == NULL) {
        printk("IPv4 addr add failed\n");
        return -EIO;
    }

    net_if_ipv4_set_netmask_by_addr(iface, &addr, &mask);

    printk("IPv4 added: %s\n",
           net_addr_ntop(AF_INET, &addr, buf, sizeof(buf)));
    printk("Netmask set: %s\n",
           net_addr_ntop(AF_INET, &mask, buf, sizeof(buf)));

    if (!net_if_ipv4_addr_lookup(&addr, &found_iface)) {
        printk("IPv4 lookup failed\n");
        return -EIO;
    }

    printk("IPv4 lookup success, found iface=%p\n", found_iface);

    if (found_iface != iface) {
        printk("IPv4 is attached to unexpected iface!\n");
        return -EIO;
    }

    return 0;
}

int main(void)
{
    int ret;
    int sock;
    int flags;
    int len;
    int64_t last_time;
    uint64_t bytes = 0;
    uint32_t packets = 0;
    uint32_t eagain_cnt = 0;
    bool first_send = true;
    struct sockaddr_in peer_addr;
    struct in_addr peer_ip;

    printk("LAN865x UDP TX bandwidth test\n");

    ret = setup_ipv4_on_lan865x();
    if (ret < 0) {
        printk("IPv4 setup failed: %d\n", ret);
        return 0;
    }

    memset(tx_buf, 0xA5, TX_SIZE);

    sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printk("socket error %d\n", errno);
        return 0;
    }

    flags = zsock_fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        printk("fcntl(F_GETFL) failed %d\n", errno);
        zsock_close(sock);
        return 0;
    }

    ret = zsock_fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        printk("fcntl(F_SETFL) failed %d\n", errno);
        zsock_close(sock);
        return 0;
    }

    printk("Socket set to non-blocking mode\n");

    memset(&peer_addr, 0, sizeof(peer_addr));
    peer_addr.sin_family = AF_INET;
    peer_addr.sin_port = htons(UDP_PORT);

    if (net_addr_pton(AF_INET, PEER_IPV4_ADDR, &peer_ip) < 0) {
        printk("Invalid peer IPv4 address string\n");
        zsock_close(sock);
        return 0;
    }

    peer_addr.sin_addr = peer_ip;

    printk("UDP TX target %s:%d payload=%d bytes\n",
           PEER_IPV4_ADDR, UDP_PORT, TX_SIZE);

    last_time = k_uptime_get();

    while (1) {
        len = zsock_sendto(sock,
                           tx_buf,
                           TX_SIZE,
                           0,
                           (struct sockaddr *)&peer_addr,
                           sizeof(peer_addr));

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                eagain_cnt++;

                {
                    int64_t now = k_uptime_get();

                    if (now - last_time >= 1000) {
                        double mbps = (bytes * 8.0) / 1000000.0;

                        printk("TX rate %.2f Mbps packets=%u bytes=%llu eagain=%u\n",
                               mbps, packets, bytes, eagain_cnt);

                        bytes = 0;
                        packets = 0;
                        eagain_cnt = 0;
                        last_time = now;
                    }
                }

                // k_sleep(K_MSEC(1));
                continue;
            }

            printk("sendto error %d\n", errno);
            k_sleep(K_MSEC(10));
            continue;
        }

        if (first_send) {
            printk("First packet sent to %s:%d len=%d\n",
                   PEER_IPV4_ADDR, UDP_PORT, len);
            first_send = false;
        }

        packets++;
        bytes += len;

        {
            int64_t now = k_uptime_get();

            if (now - last_time >= 1000) {
                double mbps = (bytes * 8.0) / 1000000.0;

                printk("TX rate %.2f Mbps packets=%u bytes=%llu eagain=%u\n",
                       mbps, packets, bytes, eagain_cnt);

                bytes = 0;
                packets = 0;
                eagain_cnt = 0;
                last_time = now;
            }
        }
    }

    zsock_close(sock);
    return 0;
}