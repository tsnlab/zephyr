#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <string.h>

#define TCP_PORT        5000
#define BUF_SIZE        1024
#define MY_IPV4_ADDR    "10.1.1.5"
#define MY_IPV4_MASK    "255.255.255.0"

static uint8_t rx_buf[BUF_SIZE];

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
    int listen_sock;
    int client_sock;
    struct sockaddr_in local_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    char client_ip[NET_IPV4_ADDR_LEN];

    printk("LAN865x TCP server test\n");

    ret = setup_ipv4_on_lan865x();
    if (ret < 0) {
        printk("IPv4 setup failed: %d\n", ret);
        return 0;
    }

    listen_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        printk("socket error %d\n", errno);
        return 0;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(TCP_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;

    ret = zsock_bind(listen_sock,
                     (struct sockaddr *)&local_addr,
                     sizeof(local_addr));
    if (ret < 0) {
        printk("bind failed %d\n", errno);
        zsock_close(listen_sock);
        return 0;
    }

    ret = zsock_listen(listen_sock, 1);
    if (ret < 0) {
        printk("listen failed %d\n", errno);
        zsock_close(listen_sock);
        return 0;
    }

    printk("TCP server listening on port %d\n", TCP_PORT);

    while (1) {
        client_addr_len = sizeof(client_addr);

        client_sock = zsock_accept(listen_sock,
                                   (struct sockaddr *)&client_addr,
                                   &client_addr_len);
        if (client_sock < 0) {
            printk("accept failed %d\n", errno);
            continue;
        }

        net_addr_ntop(AF_INET, &client_addr.sin_addr,
                      client_ip, sizeof(client_ip));
        printk("Client connected from %s:%d\n",
               client_ip, ntohs(client_addr.sin_port));

        while (1) {
            int len = zsock_recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0);

            if (len < 0) {
                printk("recv failed %d\n", errno);
                break;
            }

            if (len == 0) {
                printk("Client disconnected\n");
                break;
            }

            rx_buf[len] = '\0';
            printk("Received %d bytes: %s\n", len, rx_buf);

            ret = zsock_send(client_sock, rx_buf, len, 0);
            if (ret < 0) {
                printk("send failed %d\n", errno);
                break;
            }
        }

        zsock_close(client_sock);
    }

    return 0;
}