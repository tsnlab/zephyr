#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "eth.h"
#include "lan8651.h"

static uint16_t make_arp_request_packet(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
					const uint8_t my_ip_addr[IP_LEN],
					const uint8_t target_ip_addr[IP_LEN])
{

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct arphdr_ipv4 *arp = (struct arphdr_ipv4 *)(eth + 1);

	// Fill Ethernet header
	memset(eth->h_dest, 0xFF, ETH_ALEN); // Broadcast
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(ETH_P_ARP);

	// Fill ARP header
	arp->arp.ar_hrd = sys_cpu_to_be16(1);
	arp->arp.ar_pro = sys_cpu_to_be16(ETH_P_IP);
	arp->arp.ar_hln = ETH_ALEN;
	arp->arp.ar_pln = IP_LEN;
	arp->arp.ar_op = sys_cpu_to_be16(ARPOP_REQUEST);
	memcpy(arp->sender_mac, my_mac_addr, ETH_ALEN);
	memcpy(arp->sender_ip, my_ip_addr, IP_LEN);
	memset(arp->target_mac, 0x00, ETH_ALEN); // Unknown
	memcpy(arp->target_ip, target_ip_addr, IP_LEN);

	return PACKET_SIZE_ARP;
}

static uint16_t make_arp_reply_packet(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
					const uint8_t my_ip_addr[IP_LEN],
					const uint8_t target_mac_addr[ETH_ALEN],
					const uint8_t target_ip_addr[IP_LEN])
{

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct arphdr_ipv4 *arp = (struct arphdr_ipv4 *)(eth + 1);

	// Fill Ethernet header
	memset(eth->h_dest, 0xFF, ETH_ALEN); // Broadcast
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(ETH_P_ARP);

	// Fill ARP header
	arp->arp.ar_hrd = sys_cpu_to_be16(1);
	arp->arp.ar_pro = sys_cpu_to_be16(ETH_P_IP);
	arp->arp.ar_hln = ETH_ALEN;
	arp->arp.ar_pln = IP_LEN;
	arp->arp.ar_op = sys_cpu_to_be16(ARPOP_REPLY);
	memcpy(arp->sender_mac, my_mac_addr, ETH_ALEN);
	memcpy(arp->sender_ip, my_ip_addr, IP_LEN);
	memcpy(arp->target_mac, target_mac_addr, ETH_ALEN);
	memcpy(arp->target_ip, target_ip_addr, IP_LEN);

	return PACKET_SIZE_ARP;
}

int send_arp_request(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
		     const uint8_t my_ip_addr[IP_LEN], const uint8_t target_ip_addr[IP_LEN])
{
	uint8_t packet[PACKET_SIZE_ARP];
	uint16_t length = make_arp_request_packet(packet, my_mac_addr, my_ip_addr, target_ip_addr);

	return send_packet(spi, packet, length);
}

int send_arp_reply(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
		     const uint8_t my_ip_addr[IP_LEN], const uint8_t target_mac_addr[ETH_ALEN],
		     const uint8_t target_ip_addr[IP_LEN])
{
	uint8_t packet[PACKET_SIZE_ARP];
	uint16_t length = make_arp_reply_packet(packet, my_mac_addr, my_ip_addr, target_mac_addr, target_ip_addr);

	return send_packet(spi, packet, length);
}

int receive_arp_reply(const struct spi_dt_spec *spi)
{
	uint8_t packet[MAX_PACKET_SIZE] = {
		0,
	};
	uint16_t length = 0;

	while (length == 0) {
		if (receive_packet(spi, packet, &length) < 0) {
			printk("Failed to receive ARP reply\n");
			return -1;
		}
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct arphdr_ipv4 *arp = (struct arphdr_ipv4 *)(eth + 1);

	if (eth->h_proto != sys_cpu_to_be16(ETH_P_ARP)) {
		printk("Invalid ARP packet\n");
	}

	if (arp->arp.ar_op != sys_cpu_to_be16(ARPOP_REPLY)) {
		printk("Invalid ARP reply\n");
		return -1;
	}

	printk("Reply Received from %02x:%02x:%02x:%02x:%02x:%02x (%d.%d.%d.%d)\n", eth->h_source[0], eth->h_source[1], eth->h_source[2],
	       eth->h_source[3], eth->h_source[4], eth->h_source[5], arp->sender_ip[0], arp->sender_ip[1], arp->sender_ip[2], arp->sender_ip[3]);

	/*
	printk("ARP Request: \n");
	printk("=== Ethernet Header ===\n");
	printk("Destination MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", packet[0], packet[1], packet[2],
	       packet[3], packet[4], packet[5]);
	printk("Source MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", packet[6], packet[7], packet[8],
	       packet[9], packet[10], packet[11]);

	uint16_t tx_ethertype = (packet[12] << 8) | packet[13];
	printk("EtherType: 0x%04x (%s)\n", tx_ethertype,
	       tx_ethertype == 0x0806 ? "ARP" : "Unknown");

	printk("\n=== ARP Header ===\n");
	uint16_t tx_hw_type = (packet[14] << 8) | packet[15];
	printk("Hardware Type: 0x%04x (%s)\n", tx_hw_type,
	       tx_hw_type == 0x0001 ? "Ethernet" : "Unknown");

	uint16_t tx_proto_type = (packet[16] << 8) | packet[17];
	printk("Protocol Type: 0x%04x (%s)\n", tx_proto_type,
	       tx_proto_type == 0x0800 ? "IPv4" : "Unknown");

	uint8_t tx_hw_size = packet[18];
	uint8_t tx_proto_size = packet[19];
	printk("Hardware Size: %d bytes\n", tx_hw_size);
	printk("Protocol Size: %d bytes\n", tx_proto_size);

	uint16_t tx_operation = (packet[20] << 8) | packet[21];
	printk("Operation: 0x%04x (%s)\n", tx_operation,
	       tx_operation == 0x0001   ? "Request"
	       : tx_operation == 0x0002 ? "Reply"
					: "Unknown");

	printk("\nSender MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", packet[22], packet[23], packet[24],
	       packet[25], packet[26], packet[27]);
	printk("Sender IP: %d.%d.%d.%d\n", packet[28], packet[29], packet[30], packet[31]);

	printk("Target MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", packet[32], packet[33], packet[34],
	       packet[35], packet[36], packet[37]);
	printk("Target IP: %d.%d.%d.%d\n", packet[38], packet[39], packet[40], packet[41]);
	*/

	return 0;
}

int receive_arp_request(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
		     const uint8_t my_ip_addr[IP_LEN])
{
	uint8_t packet[MAX_PACKET_SIZE] = {
		0,
	};
	uint16_t length = 0;

	while (length == 0) {
		if (receive_packet(spi, packet, &length) < 0) {
			printk("Failed to receive ARP request\n");
			return -1;
		}
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct arphdr_ipv4 *arp = (struct arphdr_ipv4 *)(eth + 1);

	if (eth->h_proto != sys_cpu_to_be16(ETH_P_ARP)) {
		printk("Invalid ARP packet\n");
	}

	if (arp->arp.ar_op != sys_cpu_to_be16(ARPOP_REQUEST)) {
		printk("Invalid ARP request\n");
		return -1;
	}

	if (memcmp(arp->target_ip, my_ip_addr, IP_LEN) != 0) {
		printk("Target IP: %d.%d.%d.%d\n", arp->target_ip[0], arp->target_ip[1], arp->target_ip[2], arp->target_ip[3]);
		// Not for us
		return 0;
	}

	printk("Request Received from %02x:%02x:%02x:%02x:%02x:%02x (%d.%d.%d.%d)\n", eth->h_source[0], eth->h_source[1], eth->h_source[2],
	       eth->h_source[3], eth->h_source[4], eth->h_source[5], arp->sender_ip[0], arp->sender_ip[1], arp->sender_ip[2], arp->sender_ip[3]);

	printk("Sending ARP Reply\n");
	send_arp_reply(spi, my_mac_addr, my_ip_addr, eth->h_source, arp->sender_ip);

	return 0;
}
