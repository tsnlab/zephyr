#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>

#define UDP_PORT         5000
#define BUF_SIZE         1600

/* RPi5 (Zephyr) IP */
#define MY_IPV4_ADDR     "10.1.1.9"
#define MY_IPV4_MASK     "255.255.255.0"

/* RPi4 (Linux) IP */
#define PEER_IPV4_ADDR   "10.1.1.10"

/*
 * 목표 TX rate (payload 기준)
 * 필요에 따라 여기만 바꾸면 됨
 */
#define TARGET_TX_MBPS   9

/*
 * pacing 주기 (ms)
 */
#define TX_TICK_MS       10

static uint8_t rx_buf[BUF_SIZE];
static uint8_t tx_buf[BUF_SIZE];

struct bw_stats {
	uint64_t tx_bytes_total;
	uint64_t rx_bytes_total;
	uint32_t tx_pkts_total;
	uint32_t rx_pkts_total;

	uint64_t tx_bytes_interval;
	uint64_t rx_bytes_interval;
	uint32_t tx_pkts_interval;
	uint32_t rx_pkts_interval;

	uint32_t tx_eagain_interval;
	uint32_t rx_eagain_interval;
};

static struct bw_stats g_stats;
static bool g_first_tx = true;
static bool g_first_rx = true;

/* 최근 RX payload 길이 */
static size_t g_last_rx_len = 0U;
/* 최근 RX가 있었는지 */
static bool g_have_rx_sample = false;

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

static void stats_print_and_reset_interval(uint32_t interval_ms)
{
	double tx_mbps = 0.0;
	double rx_mbps = 0.0;

	if (interval_ms > 0U) {
		tx_mbps = (g_stats.tx_bytes_interval * 8.0 * 1000.0) /
			  ((double)interval_ms * 1000000.0);
		rx_mbps = (g_stats.rx_bytes_interval * 8.0 * 1000.0) /
			  ((double)interval_ms * 1000000.0);
	}

	printk("TX %6.2f Mbps pkts=%5u bytes=%9llu eagain=%4u | "
	       "RX %6.2f Mbps pkts=%5u bytes=%9llu eagain=%4u | "
	       "last_rx_len=%u\n",
	       tx_mbps,
	       g_stats.tx_pkts_interval,
	       g_stats.tx_bytes_interval,
	       g_stats.tx_eagain_interval,
	       rx_mbps,
	       g_stats.rx_pkts_interval,
	       g_stats.rx_bytes_interval,
	       g_stats.rx_eagain_interval,
	       (unsigned int)g_last_rx_len);

	g_stats.tx_bytes_interval = 0;
	g_stats.rx_bytes_interval = 0;
	g_stats.tx_pkts_interval = 0;
	g_stats.rx_pkts_interval = 0;
	g_stats.tx_eagain_interval = 0;
	g_stats.rx_eagain_interval = 0;
}

int main(void)
{
	int ret;
	int sock;
	int flags;
	int len;
	int64_t last_print_time;
	int64_t last_tx_tick;
	struct sockaddr_in local_addr;
	struct sockaddr_in peer_addr;
	struct sockaddr_in src_addr;
	struct in_addr peer_ip;
	socklen_t src_addr_len;

	uint64_t tx_credit_bytes = 0;
	const uint64_t target_tx_bytes_per_sec =
		((uint64_t)TARGET_TX_MBPS * 1000000ULL) / 8ULL;

	printk("LAN865x UDP bidirectional bandwidth test\n");
	printk("Mode: continuous RX/TX, TX payload follows last RX length\n");
	printk("Target TX %d Mbps\n", TARGET_TX_MBPS);

	ret = setup_ipv4_on_lan865x();
	if (ret < 0) {
		printk("IPv4 setup failed: %d\n", ret);
		return 0;
	}

	memset(&g_stats, 0, sizeof(g_stats));
	memset(tx_buf, 0xA5, sizeof(tx_buf));

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

	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(UDP_PORT);

	ret = net_addr_pton(AF_INET, MY_IPV4_ADDR, &local_addr.sin_addr);
	if (ret < 0) {
		printk("Invalid local IPv4 address string\n");
		zsock_close(sock);
		return 0;
	}

	ret = zsock_bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
	if (ret < 0) {
		printk("bind error %d\n", errno);
		zsock_close(sock);
		return 0;
	}

	memset(&peer_addr, 0, sizeof(peer_addr));
	peer_addr.sin_family = AF_INET;
	peer_addr.sin_port = htons(UDP_PORT);

	if (net_addr_pton(AF_INET, PEER_IPV4_ADDR, &peer_ip) < 0) {
		printk("Invalid peer IPv4 address string\n");
		zsock_close(sock);
		return 0;
	}

	peer_addr.sin_addr = peer_ip;

	printk("Local  %s:%d\n", MY_IPV4_ADDR, UDP_PORT);
	printk("Peer   %s:%d\n", PEER_IPV4_ADDR, UDP_PORT);

	last_print_time = k_uptime_get();
	last_tx_tick = last_print_time;

	while (1) {
		int64_t now = k_uptime_get();

		/* ===== RX drain ===== */
		while (1) {
			src_addr_len = sizeof(src_addr);
			len = zsock_recvfrom(sock,
					     rx_buf,
					     sizeof(rx_buf),
					     0,
					     (struct sockaddr *)&src_addr,
					     &src_addr_len);

			if (len < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					g_stats.rx_eagain_interval++;
					break;
				}

				printk("recvfrom error %d\n", errno);
				break;
			}

			if (g_first_rx) {
				char addrbuf[NET_IPV4_ADDR_LEN];

				printk("First RX packet from %s:%d len=%d\n",
				       net_addr_ntop(AF_INET,
						     &src_addr.sin_addr,
						     addrbuf,
						     sizeof(addrbuf)),
				       ntohs(src_addr.sin_port),
				       len);
				g_first_rx = false;
			}

			g_stats.rx_pkts_total++;
			g_stats.rx_bytes_total += len;
			g_stats.rx_pkts_interval++;
			g_stats.rx_bytes_interval += len;

			/* 최근 RX 길이와 데이터를 TX용으로 저장 */
			g_last_rx_len = (size_t)len;
			memcpy(tx_buf, rx_buf, len);
			g_have_rx_sample = true;
		}

		/* ===== TX pacing ===== */
		if (g_have_rx_sample && g_last_rx_len > 0U &&
		    ((now - last_tx_tick) >= TX_TICK_MS)) {
			int64_t delta_ms = now - last_tx_tick;

			tx_credit_bytes +=
				(target_tx_bytes_per_sec * (uint64_t)delta_ms) / 1000ULL;

			last_tx_tick = now;

			while (tx_credit_bytes >= g_last_rx_len) {
				len = zsock_sendto(sock,
						   tx_buf,
						   g_last_rx_len,
						   0,
						   (struct sockaddr *)&peer_addr,
						   sizeof(peer_addr));

				if (len < 0) {
					if (errno == EAGAIN || errno == EWOULDBLOCK) {
						g_stats.tx_eagain_interval++;
						break;
					}

					printk("sendto error %d\n", errno);
					break;
				}

				if (g_first_tx) {
					printk("First TX packet sent to %s:%d len=%d\n",
					       PEER_IPV4_ADDR, UDP_PORT, len);
					g_first_tx = false;
				}

				g_stats.tx_pkts_total++;
				g_stats.tx_bytes_total += len;
				g_stats.tx_pkts_interval++;
				g_stats.tx_bytes_interval += len;

				tx_credit_bytes -= g_last_rx_len;
			}
		}

		/* ===== interval 통계 출력 ===== */
		if ((now - last_print_time) >= 1000) {
			uint32_t interval_ms = (uint32_t)(now - last_print_time);

			stats_print_and_reset_interval(interval_ms);
			last_print_time = now;
		}

		k_sleep(K_MSEC(1));
	}

	zsock_close(sock);
	return 0;
}