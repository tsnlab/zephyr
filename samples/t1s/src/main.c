#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/arch/cpu.h>

#include "eth.h"
#include "lan8651.h"
#include "arp.h"
#include "perf.h"

#define THROUGHPUT_DURATION     10
#define THROUGHPUT_WARMUP       0
#define THROUGHPUT_PAYLOAD_SIZE 1500

#define SPI_NODE DT_ALIAS(spi0)

struct spi_dt_spec spi_dev;

const uint8_t my_mac_addr[ETH_ALEN] = {
	0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
};
const uint8_t target_mac_addr[ETH_ALEN] = {0xd0, 0xd1, 0x95, 0x30, 0x23, 0x01};
const uint8_t my_ip_addr[IP_LEN] = {
	10,
	1,
	1,
	2,
};
const uint8_t target_ip_addr[IP_LEN] = {
	10,
	1,
	1,
	1,
};

const uint8_t sender_mac_addr[ETH_ALEN] = { 0xaa, 0xbb, 0xcc, 0x00, 0x00, 0x33, };
// const uint8_t sender_mac_addr[ETH_ALEN] = { 0xaa, 0xbb, 0xcc, 0x00, 0x00, 0x22, };
const uint8_t receiver_mac_addr[ETH_ALEN] = {0xaa, 0xbb, 0xcc, 0x00, 0x00, 0x44, };
// const uint8_t receiver_mac_addr[ETH_ALEN] = {0xd0, 0xd1, 0x95, 0x30, 0x23, 0x00, };
const uint8_t sender_ip_addr[IP_LEN] = { 10, 1, 1, 1, };
const uint8_t receiver_ip_addr[IP_LEN] = { 10, 1, 1, 2, };

static void arp_test()
{
	// send_arp_request(&spi_dev, my_mac_addr, my_ip_addr, target_ip_addr);
	// receive_arp_reply(&spi_dev);
}

static void throughput_test()
{
	uint8_t payload[MAX_PERF_PAYLOAD] = {0};
	uint16_t len = 0;
	printk("Sending Perf Request to %02x:%02x:%02x:%02x:%02x:%02x\n", receiver_mac_addr[0], receiver_mac_addr[1], receiver_mac_addr[2], receiver_mac_addr[3], receiver_mac_addr[4], receiver_mac_addr[5]);
	// send_perf_req_packet(&spi_dev, my_mac_addr, target_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
	// while (true) {
	// 	send_perf_req_packet(&spi_dev, sender_mac_addr, receiver_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
	// 	k_busy_wait(100000);
	// }
	send_perf_req_packet(&spi_dev, sender_mac_addr, receiver_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
	// len = make_perf_data_packet(my_mac_addr, target_mac_addr, payload, THROUGHPUT_PAYLOAD_SIZE);
	len = make_perf_data_packet(sender_mac_addr, receiver_mac_addr, payload, THROUGHPUT_PAYLOAD_SIZE);
	struct ethhdr *eth = (struct ethhdr *)payload;
	struct perf_header *perf = (struct perf_header *)(eth + 1);
	uint32_t i = 0;
	int64_t start_time = k_uptime_get();
	while (true) {
		perf->id = sys_cpu_to_be32(i);
		send_perf_data_packet(&spi_dev, payload, len);
		i++;
		// k_busy_wait(1000000);
		// printk("%u\n", i);
		int64_t current_time = k_uptime_get();
		int64_t elapsed_time = current_time - start_time;
		if (elapsed_time > THROUGHPUT_DURATION * 1000) {
			printk("Throughput test finished\n");
			break;
		}
	}
	// while (true) {
	// 	perf->op = 0xff;
	// 	send_perf_data_packet(&spi_dev, payload, len);
	// }
}

static void latency_test()
{
	uint32_t id = 0;
	uint16_t len = 0;
	uint8_t client_mac_addr[ETH_ALEN];
	uint8_t payload[MAX_PERF_PAYLOAD] = {0};

	while (true) {
		while (id == 0) {
			recv_latency_req(&spi_dev, client_mac_addr, &id);
		}
		len = make_latency_res(payload, my_mac_addr, client_mac_addr, id);
		send_latency_res(&spi_dev, payload, len);
		id = 0;
	}
}

static void arp_receiver() {
	printk("ARP Receiver started\n");
	while (true) {
		receive_arp_request(&spi_dev, receiver_mac_addr, receiver_ip_addr);
	}
}

static void arp_sender() {
	printk("ARP Sender started\n");
	uint32_t seq = 0;
	while (true) {
		printk("Sending ARP Request to %d.%d.%d.%d\n", receiver_ip_addr[0], receiver_ip_addr[1], receiver_ip_addr[2], receiver_ip_addr[3]);
		uint64_t start_cycle = sys_clock_cycle_get_32();
		send_arp_request(&spi_dev, sender_mac_addr, sender_ip_addr, receiver_ip_addr, seq);
		receive_arp_reply(&spi_dev, &seq);
		uint64_t end_cycle = sys_clock_cycle_get_32();
		printk("Seq: %u, RTT: %llu ns\n", seq, (end_cycle - start_cycle) * 83);
		seq++;
		// for (int i = 0; i < 100000000; i++) {
		// 	arch_nop();
		// }
		k_busy_wait(50000);
	}
}

static void perf_server() {
	printk("Perf Server started\n");

	uint8_t source_mac_addr[ETH_ALEN] = {0};
	uint32_t duration = 0;
	uint32_t warmup = 0;

	while (duration == 0) {
		recv_perf_req(&spi_dev, source_mac_addr, &duration, &warmup);
	}
	printk("Receiving from %02x:%02x:%02x:%02x:%02x:%02x\n", source_mac_addr[0], source_mac_addr[1], source_mac_addr[2], source_mac_addr[3], source_mac_addr[4], source_mac_addr[5]);
	// send_perf_res_packet(&spi_dev, receiver_mac_addr, source_mac_addr);

	uint64_t received_bits = 0;
	uint64_t received_packets = 0;
	uint32_t last_id = 0;
	uint64_t lost_packets = 0;

	int64_t start_time = k_uptime_get();
	// printk("Start time: %lld\n", start_time);

	uint32_t elapsed_seconds = 1;
	int ret = 0;
	while (true) {
		uint32_t len = 0;
		uint32_t id = 0;
		ret = recv_perf_data(&spi_dev, source_mac_addr, &id, &len);
		// if (ret == 0xff) {
		// 	printk("End of throughput test\n");
		// 	printk("Test duration: %d seconds\n", duration);
		// 	printk("Received %llu packets, %llu bytes\n", received_packets, received_bytes);
		// 	break;
		// }
		if (len != 0) {
			lost_packets += (id - last_id) - 1;
			last_id = id;
			received_bits += len * 8;
			received_packets++;
		}
			// printk("%u\n", id);

		int64_t current_time = k_uptime_get();
		int64_t elapsed_time = current_time - start_time;
		if (elapsed_time > 1000) {
			printk("%us: %lld pps, %lld bps, loss: %.2f%%\n", elapsed_seconds, received_packets, received_bits, ((double)lost_packets / (lost_packets + received_packets)) * 100);
			elapsed_seconds++;
			// if (elapsed_seconds > duration) {
			// 	break;
			// }
			start_time = current_time;
			received_bits = 0;
			received_packets = 0;
			lost_packets = 0;
		}
	}
}

int main(void)
{
	spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
	spi_dev.config.frequency = 25000000;
	spi_dev.config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;

	set_register(&spi_dev, PLCA_MODE_COORDINATOR);
	// set_register(&spi_dev, PLCA_MODE_FOLLOWER);

	// arp_test();
	// throughput_test();
	// latency_test();
	// arp_sender();
	// arp_receiver();
	perf_server();

	return 0;
}
