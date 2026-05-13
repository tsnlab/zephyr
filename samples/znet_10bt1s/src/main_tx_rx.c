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
#define MY_IPV4_ADDR     "10.1.1.5"
#define MY_IPV4_MASK     "255.255.255.0"

/* RPi4 (Linux) IP */
#define PEER_IPV4_ADDR   "10.1.1.10"

/*
 * Fixed TX payload size for balanced-mode testing.
 */
#define TX_PAYLOAD_SIZE  1472

/*
 * Target TX rate in Mbps, based on UDP payload bytes.
 */
#define TARGET_TX_MBPS   10

/*
 * TX pacing period.
 */
#define TX_TICK_MS       10

/*
 * Thread sleep periods.
 */
#define TX_THREAD_SLEEP_MS       1
#define RX_THREAD_SLEEP_MS       1

/*
 * Per-wake budgets.
 *
 * TX_BUDGET_PER_WAKE:
 *   Maximum number of packets the TX thread can send per wake-up.
 *
 * RX_BUDGET_PER_WAKE:
 *   Maximum number of packets the RX thread can drain per wake-up.
 */
#define TX_BUDGET_PER_WAKE       4
#define RX_BUDGET_PER_WAKE       16

/*
 * RX warm-up time before starting continuous TX.
 *
 * During this period, only the RX thread is actively receiving.
 * If another node is already transmitting, RX counters should increase
 * before TX starts.
 */
#define RX_WARMUP_MS             2000

/*
 * If TX is active and no RX packet is received for this time,
 * count it as possible RX starvation.
 */
#define RX_STARVE_WARN_MS        1000

/*
 * Stack size.
 *
 * Keep this small enough to avoid RAM overflow on the current build.
 */
#define TX_THREAD_STACK_SIZE     2048
#define RX_THREAD_STACK_SIZE     2048

#define TX_THREAD_PRIO           7
#define RX_THREAD_PRIO           7

static uint8_t rx_buf[BUF_SIZE];
static uint8_t tx_buf[BUF_SIZE];

static int g_sock = -1;
static struct sockaddr_in g_peer_addr;

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

static struct k_mutex stats_lock;
static struct k_sem tx_start_sem;

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

	uint32_t tx_error_interval;
	uint32_t rx_error_interval;

	uint32_t rx_starve_interval;
	uint32_t max_rx_gap_ms;
	uint32_t last_rx_gap_ms;
};

static struct bw_stats g_stats;

static bool g_first_tx = true;
static bool g_first_rx = true;

static size_t g_last_rx_len = 0U;
static int64_t g_last_rx_time;
static bool g_tx_active;

static volatile bool g_threads_run;

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
	struct bw_stats snapshot;
	size_t last_rx_len_snapshot;
	double tx_mbps = 0.0;
	double rx_mbps = 0.0;

	k_mutex_lock(&stats_lock, K_FOREVER);

	snapshot = g_stats;
	last_rx_len_snapshot = g_last_rx_len;

	g_stats.tx_bytes_interval = 0;
	g_stats.rx_bytes_interval = 0;
	g_stats.tx_pkts_interval = 0;
	g_stats.rx_pkts_interval = 0;
	g_stats.tx_eagain_interval = 0;
	g_stats.rx_eagain_interval = 0;
	g_stats.tx_error_interval = 0;
	g_stats.rx_error_interval = 0;
	g_stats.rx_starve_interval = 0;
	g_stats.max_rx_gap_ms = 0;
	g_stats.last_rx_gap_ms = 0;

	k_mutex_unlock(&stats_lock);

	if (interval_ms > 0U) {
		tx_mbps = (snapshot.tx_bytes_interval * 8.0 * 1000.0) /
			  ((double)interval_ms * 1000000.0);
		rx_mbps = (snapshot.rx_bytes_interval * 8.0 * 1000.0) /
			  ((double)interval_ms * 1000000.0);
	}

	printk("TX %6.2f Mbps pkts=%5u bytes=%9llu eagain=%4u err=%3u | "
	       "RX %6.2f Mbps pkts=%5u bytes=%9llu eagain=%4u err=%3u | "
	       "last_rx_len=%u rx_gap=%u max_gap=%u starve=%u\n",
	       tx_mbps,
	       snapshot.tx_pkts_interval,
	       (unsigned long long)snapshot.tx_bytes_interval,
	       snapshot.tx_eagain_interval,
	       snapshot.tx_error_interval,
	       rx_mbps,
	       snapshot.rx_pkts_interval,
	       (unsigned long long)snapshot.rx_bytes_interval,
	       snapshot.rx_eagain_interval,
	       snapshot.rx_error_interval,
	       (unsigned int)last_rx_len_snapshot,
	       snapshot.last_rx_gap_ms,
	       snapshot.max_rx_gap_ms,
	       snapshot.rx_starve_interval);
}

static void tx_thread_fn(void *p1, void *p2, void *p3)
{
	uint64_t tx_credit_bytes = 0;
	const uint64_t target_tx_bytes_per_sec =
		((uint64_t)TARGET_TX_MBPS * 1000000ULL) / 8ULL;
	int64_t last_tx_tick;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("TX thread waiting for warm-up completion...\n");

	/*
	 * TX starts after the RX warm-up period.
	 *
	 * RX does not need to receive the first packet before TX starts.
	 * This allows the test to detect whether RX packets reach the socket
	 * before and after TX becomes active.
	 */
	k_sem_take(&tx_start_sem, K_FOREVER);

	printk("TX thread started\n");

	last_tx_tick = k_uptime_get();

	while (g_threads_run) {
		int64_t now = k_uptime_get();
		uint32_t tx_sent_this_wake = 0U;

		if ((now - last_tx_tick) >= TX_TICK_MS) {
			int64_t delta_ms = now - last_tx_tick;

			tx_credit_bytes +=
				(target_tx_bytes_per_sec * (uint64_t)delta_ms) / 1000ULL;

			last_tx_tick = now;
		}

		while ((tx_credit_bytes >= TX_PAYLOAD_SIZE) &&
		       (tx_sent_this_wake < TX_BUDGET_PER_WAKE)) {
			int len;

			len = zsock_sendto(g_sock,
					   tx_buf,
					   TX_PAYLOAD_SIZE,
					   0,
					   (struct sockaddr *)&g_peer_addr,
					   sizeof(g_peer_addr));

			if (len < 0) {
				k_mutex_lock(&stats_lock, K_FOREVER);

				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					g_stats.tx_eagain_interval++;
				} else {
					g_stats.tx_error_interval++;
					printk("sendto error %d\n", errno);
				}

				k_mutex_unlock(&stats_lock);
				break;
			}

			if (g_first_tx) {
				printk("First TX packet sent to %s:%d len=%d\n",
				       PEER_IPV4_ADDR, UDP_PORT, len);
				g_first_tx = false;
			}

			k_mutex_lock(&stats_lock, K_FOREVER);

			g_tx_active = true;

			g_stats.tx_pkts_total++;
			g_stats.tx_bytes_total += len;
			g_stats.tx_pkts_interval++;
			g_stats.tx_bytes_interval += len;

			k_mutex_unlock(&stats_lock);

			tx_credit_bytes -= TX_PAYLOAD_SIZE;
			tx_sent_this_wake++;
		}

		k_sleep(K_MSEC(TX_THREAD_SLEEP_MS));
	}
}

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	struct sockaddr_in src_addr;
	socklen_t src_addr_len;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (g_threads_run) {
		uint32_t rx_drained_this_wake = 0U;

		while (rx_drained_this_wake < RX_BUDGET_PER_WAKE) {
			int len;

			src_addr_len = sizeof(src_addr);

			len = zsock_recvfrom(g_sock,
					     rx_buf,
					     sizeof(rx_buf),
					     0,
					     (struct sockaddr *)&src_addr,
					     &src_addr_len);

			if (len < 0) {
				k_mutex_lock(&stats_lock, K_FOREVER);

				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					g_stats.rx_eagain_interval++;
				} else {
					g_stats.rx_error_interval++;
					printk("recvfrom error %d\n", errno);
				}

				k_mutex_unlock(&stats_lock);
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

			k_mutex_lock(&stats_lock, K_FOREVER);

			g_stats.rx_pkts_total++;
			g_stats.rx_bytes_total += len;
			g_stats.rx_pkts_interval++;
			g_stats.rx_bytes_interval += len;

			g_last_rx_len = (size_t)len;
			g_last_rx_time = k_uptime_get();

			k_mutex_unlock(&stats_lock);

			rx_drained_this_wake++;
		}

		k_sleep(K_MSEC(RX_THREAD_SLEEP_MS));
	}
}

int main(void)
{
	int ret;
	int flags;
	int64_t last_print_time;
	struct sockaddr_in local_addr;
	struct in_addr peer_ip;

	printk("LAN865x UDP balanced bandwidth test\n");
	printk("Mode: RX warm-up, then separate TX thread + RX thread\n");
	printk("TX payload=%d bytes, Target TX=%d Mbps\n",
	       TX_PAYLOAD_SIZE, TARGET_TX_MBPS);
	printk("TX budget=%d, RX budget=%d, RX warm-up=%d ms\n",
	       TX_BUDGET_PER_WAKE,
	       RX_BUDGET_PER_WAKE,
	       RX_WARMUP_MS);

	ret = setup_ipv4_on_lan865x();
	if (ret < 0) {
		printk("IPv4 setup failed: %d\n", ret);
		return 0;
	}

	k_mutex_init(&stats_lock);
	k_sem_init(&tx_start_sem, 0, 1);

	memset(&g_stats, 0, sizeof(g_stats));
	memset(tx_buf, 0xA5, sizeof(tx_buf));

	g_last_rx_time = k_uptime_get();
	g_last_rx_len = 0U;
	g_tx_active = false;

	g_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (g_sock < 0) {
		printk("socket error %d\n", errno);
		return 0;
	}

	flags = zsock_fcntl(g_sock, F_GETFL, 0);
	if (flags < 0) {
		printk("fcntl(F_GETFL) failed %d\n", errno);
		zsock_close(g_sock);
		return 0;
	}

	ret = zsock_fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);
	if (ret < 0) {
		printk("fcntl(F_SETFL) failed %d\n", errno);
		zsock_close(g_sock);
		return 0;
	}

	printk("Socket set to non-blocking mode\n");

	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(UDP_PORT);

	ret = net_addr_pton(AF_INET, MY_IPV4_ADDR, &local_addr.sin_addr);
	if (ret < 0) {
		printk("Invalid local IPv4 address string\n");
		zsock_close(g_sock);
		return 0;
	}

	ret = zsock_bind(g_sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
	if (ret < 0) {
		printk("bind error %d\n", errno);
		zsock_close(g_sock);
		return 0;
	}

	memset(&g_peer_addr, 0, sizeof(g_peer_addr));
	g_peer_addr.sin_family = AF_INET;
	g_peer_addr.sin_port = htons(UDP_PORT);

	if (net_addr_pton(AF_INET, PEER_IPV4_ADDR, &peer_ip) < 0) {
		printk("Invalid peer IPv4 address string\n");
		zsock_close(g_sock);
		return 0;
	}

	g_peer_addr.sin_addr = peer_ip;

	printk("Local  %s:%d\n", MY_IPV4_ADDR, UDP_PORT);
	printk("Peer   %s:%d\n", PEER_IPV4_ADDR, UDP_PORT);
	printk("Starting RX thread first, then TX thread after warm-up\n");

	g_threads_run = true;

	/*
	 * Start the RX thread first.
	 *
	 * This gives the socket receive path and the LAN865x polling path
	 * time to observe incoming traffic before continuous TX starts.
	 */
	k_thread_create(&rx_thread_data,
			rx_thread_stack,
			K_THREAD_STACK_SIZEOF(rx_thread_stack),
			rx_thread_fn,
			NULL,
			NULL,
			NULL,
			K_PRIO_PREEMPT(RX_THREAD_PRIO),
			0,
			K_NO_WAIT);

	k_thread_name_set(&rx_thread_data, "udp_rx_test");

	/*
	 * Start the TX thread now.
	 *
	 * The TX thread blocks on tx_start_sem until the warm-up period is
	 * complete.
	 */
	k_thread_create(&tx_thread_data,
			tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack),
			tx_thread_fn,
			NULL,
			NULL,
			NULL,
			K_PRIO_PREEMPT(TX_THREAD_PRIO),
			0,
			K_NO_WAIT);

	k_thread_name_set(&tx_thread_data, "udp_tx_test");

	last_print_time = k_uptime_get();

	/*
	 * RX warm-up period.
	 *
	 * If another node is already transmitting to this board, RX counters
	 * should increase during this period. If RX remains zero, packets are
	 * not reaching the socket receive path.
	 */
	k_sleep(K_MSEC(RX_WARMUP_MS));

	printk("RX warm-up complete, starting TX thread\n");
	k_sem_give(&tx_start_sem);

	while (1) {
		int64_t now = k_uptime_get();

		if (g_tx_active) {
			uint32_t rx_gap_ms = (uint32_t)(now - g_last_rx_time);

			k_mutex_lock(&stats_lock, K_FOREVER);

			g_stats.last_rx_gap_ms = rx_gap_ms;

			if (rx_gap_ms > g_stats.max_rx_gap_ms) {
				g_stats.max_rx_gap_ms = rx_gap_ms;
			}

			if (rx_gap_ms >= RX_STARVE_WARN_MS) {
				g_stats.rx_starve_interval++;
			}

			k_mutex_unlock(&stats_lock);
		}

		if ((now - last_print_time) >= 1000) {
			uint32_t interval_ms = (uint32_t)(now - last_print_time);

			stats_print_and_reset_interval(interval_ms);
			last_print_time = now;
		}

		k_sleep(K_MSEC(100));
	}

	g_threads_run = false;
	zsock_close(g_sock);

	return 0;
}