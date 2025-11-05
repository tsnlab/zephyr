#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/arch/cpu.h>

#include "eth.h"
#include "lan8651.h"
#include "arp.h"
#include "perf.h"
#include "udp.h"

#define UDP_ECHO_SERVER_PORT 0x0007
#define UDP_THROUGHPUT_SERVER_PORT 0x1337

#define THROUGHPUT_DURATION     10
#define THROUGHPUT_WARMUP       0
#define THROUGHPUT_PAYLOAD_SIZE 1500

#define T1S_PLCA_NODE_COUNT 0x08

#define SPI_NODE DT_ALIAS(spi0)

static const struct device *uart1 = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

struct spi_dt_spec spi_dev;

uint8_t my_mac_addr[ETH_ALEN] = {
	0xCC, 0x00, 0xFF, 0xEE, 0x00, 0x00, 0x00,
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

static void print_help() {
	printk("Available commands:\n");
	printk("help\n");
	printk("t1s_init <node_id>\n");
	printk("udp_echo_client <source_port> <interval_ms>\n");
	printk("udp_echo_server\n");
	printk("udp_throughput_client <source_port> <duration_ms>\n");
	printk("udp_throughput_server\n");
}

static void t1s_init(uint8_t node_id)
{
	spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
	spi_dev.config.frequency = 25000000;
	spi_dev.config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;

	/* Set MAC Address */
	my_mac_addr[5] = node_id;

	/* Initialize T1S */
	set_register(&spi_dev, T1S_PLCA_NODE_COUNT, node_id, my_mac_addr);

	printk("T1S initialized with node ID: %u(%s), MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", node_id, node_id == 0 ? "coordinator" : "follower", 
		my_mac_addr[0], my_mac_addr[1], my_mac_addr[2], my_mac_addr[3], my_mac_addr[4], my_mac_addr[5]);
}

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

static void udp_echo_client(uint16_t source_port, uint32_t interval_ms) {
	uint32_t send_seq = 0;
	uint16_t received_udp_packet_length = 0;
	uint8_t received_udp_src_mac_addr[ETH_ALEN] = {0};
	uint16_t received_udp_src_port = 0;
	uint8_t received_data[12];

	printk("UDP Echo Client started\n");

	while (true) {
		int64_t send_time = k_uptime_get();

		send_udp_packet(&spi_dev, sender_mac_addr, sender_ip_addr, 
								receiver_mac_addr, receiver_ip_addr, 
								source_port, UDP_ECHO_SERVER_PORT, 
								(uint8_t *)&send_seq, sizeof(send_seq));
		printk("Sent Sequence Number: %u\n", send_seq);

		while (received_udp_packet_length == 0) {
			receive_udp_packet(&spi_dev, received_udp_src_mac_addr, &received_udp_src_port, 
									received_data, &received_udp_packet_length);
		}
		uint32_t received_seq = *(uint32_t *)received_data;
		if (received_seq != send_seq) {
			printk("Sequence number mismatch: expected %u, got %u\n", send_seq, received_seq);
			continue;
		}
		printk("Received Sequence Number: %u\n", received_seq);

		int64_t receive_time = k_uptime_get();
		printk("RTT: %lld us\n", receive_time - send_time);
		k_busy_wait(interval_ms * 1000);
		send_seq++;
	}
}

static void udp_echo_server() {
	uint8_t received_udp_src_mac_addr[ETH_ALEN] = {0};
	uint16_t received_udp_src_port = 0;
	uint8_t received_data[12] = {0};
	uint16_t received_udp_packet_length = 0;

	printk("UDP Echo Server started\n");

	while (true) {
		while (received_udp_packet_length == 0) {
			receive_udp_packet(&spi_dev, received_udp_src_mac_addr, &received_udp_src_port, 
									received_data, &received_udp_packet_length);
		}
		uint32_t received_seq = *(uint32_t *)received_data;
		printk("Received Sequence Number: %u\n", received_seq);
		send_udp_packet(&spi_dev, receiver_mac_addr, receiver_ip_addr, 
								sender_mac_addr, sender_ip_addr, 
								UDP_ECHO_SERVER_PORT, received_udp_src_port, 
								(uint8_t *)&received_seq, sizeof(received_seq));
		received_udp_packet_length = 0;
	}
}

static void udp_throughput_client(uint16_t source_port, uint32_t duration_ms) {
	printk("UDP Throughput Client started\n");
	
	uint8_t packet[MAX_PERF_PAYLOAD] = {0};
	uint16_t udp_packet_length = 0;

	/* Send Perf Request */
	udp_packet_length = make_udp_packet(packet, sender_mac_addr, sender_ip_addr, 
								receiver_mac_addr, receiver_ip_addr, 
								source_port, UDP_THROUGHPUT_SERVER_PORT, 
								NULL, 0);
	make_udp_perf_req_packet(packet + udp_packet_length, duration_ms, 0);
	send_udp_perf_req_packet(&spi_dev, packet, sizeof(packet));

	printk("Waiting for server to initialize\n");
    uint32_t wait_server_init_time_ms = 1000 * 1000; // 1 second
    k_busy_wait(wait_server_init_time_ms);

	/* Send Perf Data */
	udp_packet_length = make_udp_packet(packet, sender_mac_addr, sender_ip_addr, 
		receiver_mac_addr, receiver_ip_addr, 
		source_port, UDP_THROUGHPUT_SERVER_PORT, 
		NULL, 0);
	make_udp_perf_data_packet(packet + udp_packet_length);

	int64_t start_time = k_uptime_get();
	while(true) {
		send_udp_perf_data_packet(&spi_dev, packet, sizeof(packet));
		int64_t current_time = k_uptime_get();
		int64_t elapsed_time = current_time - start_time;
		if (elapsed_time > duration_ms * 1000) {
			break;
		}
	}

	printk("UDP Throughput Client finished\n");
}

static void udp_throughput_server() {
	printk("UDP Throughput Server started\n");

	uint8_t source_mac_addr[ETH_ALEN] = {0};
	uint8_t source_ip_addr[IP_LEN] = {0};
	uint16_t source_port = 0;
	uint32_t requested_duration_ms = 0;
	
	while (requested_duration_ms == 0) {
		recv_udp_perf_req(&spi_dev, source_mac_addr, source_ip_addr, &source_port, &requested_duration_ms);
	}
	printk("Receiving from %02x:%02x:%02x:%02x:%02x:%02x (%d.%d.%d.%d:%d)\n", 
		source_mac_addr[0], source_mac_addr[1], source_mac_addr[2], source_mac_addr[3], source_mac_addr[4], source_mac_addr[5], 
		source_ip_addr[0], source_ip_addr[1], source_ip_addr[2], source_ip_addr[3], source_port);

	uint64_t received_bits = 0;
	uint64_t received_packets = 0;
	uint32_t last_id = 0;
	uint64_t lost_packets = 0;

	int64_t start_time = k_uptime_get();

	uint32_t elapsed_seconds = 1;
	int received_length = 0;
	while (true) {
		uint32_t id = 0;
		received_length = recv_udp_perf_data(&spi_dev, source_port, &id);
		if (received_length != 0) {
			lost_packets += (id - last_id) - 1;
			last_id = id;
			received_bits += received_length * 8;
			received_packets++;
		}

		int64_t current_time = k_uptime_get();
		int64_t elapsed_time = current_time - start_time;
		if (elapsed_time > 1000) {
			printk("%us: %lld pps, %lld bps, loss: %.2f%%\n", elapsed_seconds, 
				received_packets, received_bits, ((double)lost_packets / (lost_packets + received_packets)) * 100);
			elapsed_seconds++;
			if (elapsed_seconds > requested_duration_ms / 1000) {
				break;
			}
			start_time = current_time;
			received_bits = 0;
			received_packets = 0;
			lost_packets = 0;
		}
	}

	printk("UDP Throughput Server finished\n");
}

int main(void)
{
	char command_buffer[256] = {0};
	char command_char = '\0';
	int index = 0;

	if (!device_is_ready(uart1)) {
		printk("Error: UART %s is not ready\n", uart1->name);
		return -1;
	}

	printk("Ready");

	while(1) {

		/* Read Command */
		index = 0;
		memset(command_buffer, '\0', sizeof(command_buffer));

		while (index < (sizeof(command_buffer) - 1)) {
			int ret = uart_poll_in(uart1, &command_char);
			if (ret == 0) {
				if (command_char == '\n' || command_char == '\r') {
					break;
				}
				if (index == (sizeof(command_buffer) - 1)) {
					printk("Command buffer is full: %s\n", command_buffer);
					break;
				}

				command_buffer[index++] = command_char;
			}
		}
		
		command_buffer[index] = '\0';
		if (index > 0) {
			printk("Command: %s\n", command_buffer);
		}

		/* Process Command */
		if (strcmp(command_buffer, "help") == 0) {
			print_help();
		} else if (strncmp(command_buffer, "t1s_init", 8) == 0) {
			uint8_t node_id = 0;
			sscanf(command_buffer + 9, "%hhu", &node_id);
			t1s_init(node_id);
		} else if (strncmp(command_buffer, "udp_echo_client", 15) == 0) {
			uint16_t source_port = 0;
			uint32_t interval_ms = 0;
			sscanf(command_buffer + 16, "%hu %u", &source_port, &interval_ms);
			udp_echo_client(source_port, interval_ms);
		} else if (strcmp(command_buffer, "udp_echo_server") == 0) {
			udp_echo_server();
		} else if (strncmp(command_buffer, "udp_throughput_client", 21) == 0) {
			uint16_t source_port = 0;
			uint32_t duration_ms = 0;
			sscanf(command_buffer + 22, "%hu %u", &source_port, &duration_ms);
			udp_throughput_client(source_port, duration_ms);
		} else if (strcmp(command_buffer, "udp_throughput_server") == 0) {
			udp_throughput_server();
		} else {
			printk("Unknown command: %s\n", command_buffer);
		}
	}

	return 0;
}
