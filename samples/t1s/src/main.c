#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/arch/cpu.h>

#include "eth.h"
#include "lan8651.h"
#include "arp.h"
#include "perf.h"
#include "udp.h"
#include "pubsub/pingpong.h"

#define UDP_ECHO_SERVER_PORT 0x0007
#define UDP_THROUGHPUT_SERVER_PORT 0x1337

#define THROUGHPUT_DURATION     10
#define THROUGHPUT_WARMUP       0
#define THROUGHPUT_PAYLOAD_SIZE 1500

#define T1S_PLCA_NODE_COUNT 0x08

#define SPI_NODE DT_ALIAS(spi0)

static const struct device *uart1 = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

struct spi_dt_spec spi_dev;

uint8_t my_mac_addr[ETH_ALEN];
uint8_t target_mac_addr[ETH_ALEN];
uint8_t my_ip_addr[IP_LEN];
uint8_t target_ip_addr[IP_LEN];

uint8_t tickle_node_id = 0;

static void print_help() {
	printk("Available commands:\n");
	printk("help\n");
	printk("t1s_init <node_id>\n");
	printk("udp_echo_client <source_port> <interval_ms>\n");
	printk("udp_echo_server\n");
	printk("udp_throughput_client <source_port> <duration_ms>\n");
	printk("udp_throughput_server\n");
	printk("tickle_ping\n");
	printk("tickle_pong\n");
}

static int parse_uint8(const char *str, uint8_t *out) {
	char *endptr;
	unsigned long val;

	if (str == NULL || *str == '\0') {
		return -1;
	}

	errno = 0;
	val = strtoul(str, &endptr, 10);

	if (errno != 0 || *endptr != '\0' || val > UINT8_MAX) {
		return -1;
	}

	*out = (uint8_t)val;
	return 0;
}

static int parse_uint16(const char *str, uint16_t *out) {
	char *endptr;
	unsigned long val;

	if (str == NULL || *str == '\0') {
		return -1;
	}

	errno = 0;
	val = strtoul(str, &endptr, 10);

	if (errno != 0 || *endptr != '\0' || val > UINT16_MAX) {
		return -1;
	}

	*out = (uint16_t)val;
	return 0;
}

static int parse_uint32(const char *str, uint32_t *out) {
	char *endptr;
	unsigned long val;

	if (str == NULL || *str == '\0') {
		return -1;
	}

	errno = 0;
	val = strtoul(str, &endptr, 10);

	if (errno != 0 || *endptr != '\0' || val > UINT32_MAX) {
		return -1;
	}

	*out = (uint32_t)val;
	return 0;
}

static void t1s_init(uint8_t node_id)
{
	printk("Initializing T1S\n");

	spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
	spi_dev.config.frequency = 25000000;
	spi_dev.config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;

	/* Set MAC Address */
	my_mac_addr[5] = node_id == 0x00 ? 0x00 : 0xFF;
	my_ip_addr[3] = node_id;

	/* Initialize T1S */
	set_register(&spi_dev, (uint16_t)T1S_PLCA_NODE_COUNT, node_id, my_mac_addr);

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
	printk("Sending Perf Request to %02x:%02x:%02x:%02x:%02x:%02x\n", target_mac_addr[0], target_mac_addr[1], target_mac_addr[2], target_mac_addr[3], target_mac_addr[4], target_mac_addr[5]);
	// send_perf_req_packet(&spi_dev, my_mac_addr, target_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
	// while (true) {
	// 	send_perf_req_packet(&spi_dev, my_mac_addr, target_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);	
	// 	k_busy_wait(100000);
	// }
	send_perf_req_packet(&spi_dev, my_mac_addr, target_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
	// len = make_perf_data_packet(my_mac_addr, target_mac_addr, payload, THROUGHPUT_PAYLOAD_SIZE);
	len = make_perf_data_packet(my_mac_addr, target_mac_addr, payload, THROUGHPUT_PAYLOAD_SIZE);
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
	uint8_t payload[MAX_PERF_PAYLOAD] = {0};

	while (true) {
		while (id == 0) {
			recv_latency_req(&spi_dev, target_mac_addr, &id);
		}
		len = make_latency_res(payload, my_mac_addr, target_mac_addr, id);
		send_latency_res(&spi_dev, payload, len);
		id = 0;
	}
}

static void arp_receiver() {
	printk("ARP Receiver started\n");
	while (true) {
		receive_arp_request(&spi_dev, target_mac_addr, target_ip_addr);
	}
}

static void arp_sender() {
	printk("ARP Sender started\n");
	uint32_t seq = 0;
	while (true) {
		printk("Sending ARP Request to %d.%d.%d.%d\n", target_ip_addr[0], target_ip_addr[1], target_ip_addr[2], target_ip_addr[3]);
		uint64_t start_cycle = sys_clock_cycle_get_32();
		send_arp_request(&spi_dev, my_mac_addr, my_ip_addr, target_ip_addr, seq);
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
	// send_perf_res_packet(&spi_dev, target_mac_addr, source_mac_addr);

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
	uint8_t received_udp_src_mac_addr[ETH_ALEN] = {0};
	uint8_t received_udp_src_ip_addr[IP_LEN] = {0};
	uint16_t received_udp_src_port = 0;
	uint8_t received_data[12] = {0,};
	uint32_t received_data_length = 0;

	printk("UDP Echo Client started\n");

	while (true) {
		uint32_t start_clock = sys_clock_cycle_get_32();
		uint32_t send_start_clock, send_end_clock;
		uint32_t recv_start_clock, recv_end_clock;
		uint32_t end_clock;

		// Send UDP packet
		send_start_clock = sys_clock_cycle_get_32();
		send_udp_packet(&spi_dev, my_mac_addr, my_ip_addr, 
								target_mac_addr, target_ip_addr, 
								source_port, UDP_ECHO_SERVER_PORT, 
								(uint8_t *)&send_seq, sizeof(send_seq));
		send_end_clock = sys_clock_cycle_get_32();
		uint32_t send_time_ns = (send_end_clock - send_start_clock) * 83;

		// Receive UDP packet
		recv_start_clock = sys_clock_cycle_get_32();
		while (received_data_length == 0) {
			receive_udp_packet(&spi_dev, received_udp_src_mac_addr, received_udp_src_ip_addr, &received_udp_src_port, 
									received_data, &received_data_length);
		}
		recv_end_clock = sys_clock_cycle_get_32();
		uint32_t recv_time_ns = (recv_end_clock - recv_start_clock) * 83;

		uint32_t received_seq = *(uint32_t *)received_data;
		if (received_seq != send_seq) {
			printk("Sequence number mismatch: expected %u, got %u\n", send_seq, received_seq);
			received_data_length = 0;
			continue;
		}

		end_clock = sys_clock_cycle_get_32();
		uint32_t total_rtt_ns = (end_clock - start_clock) * 83;
		printk("[Seq %u] Total RTT: %u ns (send: %u ns, recv: %u ns)\n", 
		       send_seq, total_rtt_ns, send_time_ns, recv_time_ns);

		k_busy_wait(interval_ms * 1000);
		send_seq++;

		received_data_length = 0;
	}
}

static void udp_echo_server() {
	uint8_t received_udp_src_mac_addr[ETH_ALEN] = {0};
	uint8_t received_udp_src_ip_addr[IP_LEN] = {0};
	uint16_t received_udp_src_port = 0;
	uint8_t received_data[12] = {0,};
	uint32_t received_data_length = 0;

	printk("UDP Echo Server started\n");

	while (true) {
		uint32_t process_start_clock, process_end_clock;
		uint32_t send_start_clock, send_end_clock;
		uint32_t total_start_clock, total_end_clock;

		// Receive UDP packet
		while (received_data_length == 0) {
			receive_udp_packet(&spi_dev, received_udp_src_mac_addr, received_udp_src_ip_addr, &received_udp_src_port, 
									received_data, &received_data_length);
		}

		total_start_clock = sys_clock_cycle_get_32();

		// Process received data
		process_start_clock = sys_clock_cycle_get_32();
		uint32_t received_seq = *(uint32_t *)received_data;
		process_end_clock = sys_clock_cycle_get_32();
		uint32_t process_time_ns = (process_end_clock - process_start_clock) * 83;

		// Send response UDP packet
		send_start_clock = sys_clock_cycle_get_32();
		send_udp_packet(&spi_dev, my_mac_addr, my_ip_addr, 
								received_udp_src_mac_addr, received_udp_src_ip_addr, 
								UDP_ECHO_SERVER_PORT, received_udp_src_port, 
								(uint8_t *)&received_seq, sizeof(received_seq));
		send_end_clock = sys_clock_cycle_get_32();
		uint32_t send_time_ns = (send_end_clock - send_start_clock) * 83;

		total_end_clock = sys_clock_cycle_get_32();
		uint32_t total_time_ns = (total_end_clock - total_start_clock) * 83;

		printk("[Seq %u] Total: %u ns (send: %u ns, process: %u ns)\n",
		       received_seq, total_time_ns, send_time_ns, process_time_ns);

		received_data_length = 0;
	}
}

static void udp_throughput_client(uint16_t source_port, uint32_t duration_ms) {
	printk("UDP Throughput Client started\n");
	
	uint8_t packet[MAX_PERF_PAYLOAD] = {0};
	uint16_t udp_packet_length = 0;
	uint32_t perf_id = 0;

	/* Send Perf Request */
	udp_packet_length = make_udp_packet(packet, my_mac_addr, my_ip_addr, 
								target_mac_addr, target_ip_addr, 
								source_port, UDP_THROUGHPUT_SERVER_PORT, 
								NULL, 0);
	make_udp_perf_req_packet(packet + udp_packet_length, duration_ms, 0);
	send_udp_perf_req_packet(&spi_dev, packet, sizeof(packet));

	printk("Waiting for server to initialize\n");
    uint32_t wait_server_init_time_ms = 1000 * 1000; // 1 second
    k_busy_wait(wait_server_init_time_ms);

	/* Send Perf Data */
	udp_packet_length = make_udp_packet(packet, my_mac_addr, my_ip_addr, 
		target_mac_addr, target_ip_addr, 
		source_port, UDP_THROUGHPUT_SERVER_PORT, 
		NULL, 0);
	make_udp_perf_data_packet(packet + udp_packet_length);

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
	struct udphdr *udp = (struct udphdr *)(ipv4 + 1);
	struct perf_header *perf = (struct perf_header *)(udp + 1);

	int64_t start_time = k_uptime_get();
	while(true) {
		perf->id = sys_cpu_to_be32(perf_id);
		send_udp_perf_data_packet(&spi_dev, packet, sizeof(packet));
		perf_id++;
		int64_t current_time = k_uptime_get();
		int64_t elapsed_time = current_time - start_time;
		if (elapsed_time > duration_ms) {
			break;
		}

		/* Transmission Interval Adjustment */
		k_busy_wait(408);
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
	char command_buffer[64] = {0};
	char command_char = '\0';
	int index = 0;

	if (!device_is_ready(uart1)) {
		printk("Error: UART %s is not ready\n", uart1->name);
		return -1;
	}

	memcpy(my_mac_addr, (const uint8_t[]){0xCC, 0x00, 0xFF, 0xEE, 0x00, 0x00}, ETH_ALEN);
	memcpy(my_ip_addr, (const uint8_t[]){10, 1, 1, 0}, IP_LEN);
	memcpy(target_mac_addr, (const uint8_t[]){0xCC, 0x00, 0xFF, 0xEE, 0x00, 0xFF}, ETH_ALEN);
	memcpy(target_ip_addr, (const uint8_t[]){10, 1, 1, 255}, IP_LEN);

	printk("Ready\n");

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
		} else if (strncmp(command_buffer, "t1s_init", strlen("t1s_init")) == 0) {
			char *arg_start = command_buffer + strlen("t1s_init");
			uint8_t node_id = 0;

			// Skip whitespace
			while (*arg_start == ' ' || *arg_start == '\t') {
				arg_start++;
			}

			if (*arg_start == '\0') {
				printk("Error: t1s_init requires node_id argument\n");
				continue;
			}

			// Parse node_id
			if (parse_uint8(arg_start, &node_id) != 0) {
				printk("Error: Invalid node_id format. Expected a number (0-255)\n");
				continue;
			}

			t1s_init(node_id);
			tickle_node_id = (node_id == 255 ? 0: node_id + 1);
		} else if (strncmp(command_buffer, "udp_echo_client", strlen("udp_echo_client")) == 0) {
			char *arg_start = command_buffer + strlen("udp_echo_client");
			char *token;
			char *saveptr;
			uint16_t source_port = 0;
			uint32_t interval_ms = 0;

			// Skip whitespace
			while (*arg_start == ' ' || *arg_start == '\t') {
				arg_start++;
			}

			if (*arg_start == '\0') {
				printk("Error: udp_echo_client requires source_port and interval_ms arguments\n");
				continue;
			}

			// Parse first argument (source_port)
			token = strtok_r(arg_start, " \t", &saveptr);
			if (token == NULL || parse_uint16(token, &source_port) != 0) {
				printk("Error: Invalid source_port format\n");
				continue;
			}

			// Parse second argument (interval_ms)
			token = strtok_r(NULL, " \t", &saveptr);
			if (token == NULL || parse_uint32(token, &interval_ms) != 0) {
				printk("Error: Invalid interval_ms format\n");
				continue;
			}

			udp_echo_client(source_port, interval_ms);
		} else if (strcmp(command_buffer, "udp_echo_server") == 0) {
			udp_echo_server();
		} else if (strncmp(command_buffer, "udp_throughput_client", strlen("udp_throughput_client")) == 0) {
			char *arg_start = command_buffer + strlen("udp_throughput_client");
			char *token;
			char *saveptr;
			uint16_t source_port = 0;
			uint32_t duration_ms = 0;

			// Skip whitespace
			while (*arg_start == ' ' || *arg_start == '\t') {
				arg_start++;
			}

			if (*arg_start == '\0') {
				printk("Error: udp_throughput_client requires source_port and duration_ms arguments\n");
				continue;
			}

			// Parse first argument (source_port)
			token = strtok_r(arg_start, " \t", &saveptr);
			if (token == NULL || parse_uint16(token, &source_port) != 0) {
				printk("Error: Invalid source_port format\n");
				continue;
			}

			// Parse second argument (duration_ms)
			token = strtok_r(NULL, " \t", &saveptr);
			if (token == NULL || parse_uint32(token, &duration_ms) != 0) {
				printk("Error: Invalid duration_ms format\n");
				continue;
			}

			udp_throughput_client(source_port, duration_ms);
		} else if (strcmp(command_buffer, "udp_throughput_server") == 0) {
			udp_throughput_server();
		} else if (strcmp(command_buffer, "tickle_ping") == 0) {
			ping_main();
		} else if (strcmp(command_buffer, "tickle_pong") == 0) {
			pong_main();
		} else {
			printk("Unknown command: %s\n", command_buffer);
		}
	}

	return 0;
}
