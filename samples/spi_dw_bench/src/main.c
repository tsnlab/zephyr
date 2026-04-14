#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#define PAYLOAD_SIZE 1472U
#define PEER_IP_STR  "10.1.1.10"
#define PEER_PORT    5000

static uint8_t tx_buf[PAYLOAD_SIZE];

static int wait_for_iface_up(struct net_if *iface, int timeout_ms)
{
	int waited_ms = 0;

	while (waited_ms < timeout_ms) {
		if ((iface != NULL) && net_if_is_up(iface)) {
			return 0;
		}

		k_sleep(K_MSEC(100));
		waited_ms += 100;
	}

	return -ETIMEDOUT;
}

int main(void)
{
	struct net_if *iface;
	struct sockaddr_in peer_addr;
	int sock;
	int ret;

	uint64_t total_bytes = 0U;
	uint64_t total_pkts = 0U;
	uint64_t total_send_err = 0U;

	uint64_t window_bytes = 0U;
	uint64_t window_pkts = 0U;
	uint64_t window_send_err = 0U;

	int64_t window_start_ms;
	int64_t total_start_ms;

	printk("Hello World! eth_lan865x tx path continuous sender start\n");
	printk("  PAYLOAD_SIZE=%u\n", PAYLOAD_SIZE);
	printk("  PEER=%s:%u\n", PEER_IP_STR, PEER_PORT);

	/* Allow driver and interface initialization to complete. */
	k_sleep(K_SECONDS(2));

	iface = net_if_get_default();
	if (iface == NULL) {
		printk("default network interface is NULL\n");
		return 0;
	}

	ret = wait_for_iface_up(iface, 5000);
	if (ret < 0) {
		printk("network interface is not up\n");
		return 0;
	}

	for (size_t i = 0; i < PAYLOAD_SIZE; i++) {
		tx_buf[i] = (uint8_t)(i & 0xff);
	}

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printk("socket() failed: %d\n", errno);
		return 0;
	}

	memset(&peer_addr, 0, sizeof(peer_addr));
	peer_addr.sin_family = AF_INET;
	peer_addr.sin_port = htons(PEER_PORT);

	ret = zsock_inet_pton(AF_INET, PEER_IP_STR, &peer_addr.sin_addr);
	if (ret != 1) {
		printk("inet_pton() failed for peer ip %s\n", PEER_IP_STR);
		zsock_close(sock);
		return 0;
	}

	window_start_ms = k_uptime_get();
	total_start_ms = window_start_ms;

	while (1) {
		ssize_t sent;
		int64_t now_ms;
		uint64_t total_elapsed_ms;
		uint64_t window_elapsed_ms;

		sent = zsock_sendto(sock,
				    tx_buf,
				    PAYLOAD_SIZE,
				    0,
				    (const struct sockaddr *)&peer_addr,
				    sizeof(peer_addr));

		if (sent < 0) {
			total_send_err++;
			window_send_err++;

			printk("sendto() failed: errno=%d\n", errno);

			/*
			 * Do not exit on transient failures.
			 * Keep the sender running so the remote node can observe
			 * whether traffic resumes after a short delay.
			 */
			k_sleep(K_MSEC(10));
		} else if ((size_t)sent != PAYLOAD_SIZE) {
			total_send_err++;
			window_send_err++;

			printk("partial send: sent=%d expected=%u\n",
			       (int)sent, PAYLOAD_SIZE);

			k_sleep(K_MSEC(10));
		} else {
			total_bytes += PAYLOAD_SIZE;
			total_pkts++;

			window_bytes += PAYLOAD_SIZE;
			window_pkts++;
		}

		now_ms = k_uptime_get();
		window_elapsed_ms = (uint64_t)(now_ms - window_start_ms);

		if (window_elapsed_ms >= 1000U) {
			uint64_t total_mbps_x100;
			uint64_t window_mbps_x100;

			total_elapsed_ms = (uint64_t)(now_ms - total_start_ms);

			if (total_elapsed_ms == 0U) {
				total_mbps_x100 = 0U;
			} else {
				total_mbps_x100 =
					(total_bytes * 8ULL * 100ULL) / total_elapsed_ms / 1000ULL;
			}

			window_mbps_x100 =
				(window_bytes * 8ULL * 100ULL) / window_elapsed_ms / 1000ULL;

			printk("app send loop: window_pkts=%llu window_bytes=%llu window_err=%llu "
			       "window=%llu.%02llu Mbps total_pkts=%llu total_bytes=%llu total_err=%llu "
			       "total=%llu.%02llu Mbps\n",
			       window_pkts,
			       window_bytes,
			       window_send_err,
			       window_mbps_x100 / 100ULL,
			       window_mbps_x100 % 100ULL,
			       total_pkts,
			       total_bytes,
			       total_send_err,
			       total_mbps_x100 / 100ULL,
			       total_mbps_x100 % 100ULL);

			window_bytes = 0U;
			window_pkts = 0U;
			window_send_err = 0U;
			window_start_ms = now_ms;
		}
	}

	/* Unreachable in the continuous sender path. */
	zsock_close(sock);
	return 0;
}