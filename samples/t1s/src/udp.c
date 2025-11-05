#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "eth.h"
#include "lan8651.h"
#include "udp.h"

static uint16_t csum16(const void *data, size_t len) {
    uint32_t sum = 0;
    const uint16_t *p = (const uint16_t *)data;
    while (len > 1) { sum += *p++; len -= 2; }
    if (len == 1) {
        uint16_t last = 0;
        *(uint8_t*)(&last) = *(const uint8_t*)p;
        sum += last;
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}


uint16_t make_udp_packet(uint8_t *packet, const uint8_t my_mac_addr[ETH_ALEN],
					const uint8_t my_ip_addr[IP_LEN], const uint8_t target_mac_addr[ETH_ALEN],
					const uint8_t target_ip_addr[IP_LEN], uint16_t source_port, uint16_t target_port, 
					const uint8_t *data, uint16_t data_length)
{

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
	struct udphdr *udp = (struct udphdr *)(ipv4 + 1);

	// Fill Ethernet header
	memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
	memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(ETH_P_IP);

	// Fill ARP header
	ipv4->version = 4;  /* IPv4 */
	ipv4->ihl = 5;  /* no options : 5 * 4 = 20 bytes */
	ipv4->tos = 0;
	ipv4->len = sys_cpu_to_be16(sizeof(struct ipv4hdr) + sizeof(struct udphdr) + data_length);
	ipv4->id = 0;
	ipv4->frag_off = 0;
	ipv4->ttl = 64;
	ipv4->proto = IPPROTO_UDP;
	memcpy(&ipv4->src, my_ip_addr, IP_LEN);
	memcpy(&ipv4->dst, target_ip_addr, IP_LEN);
	ipv4->chksum = csum16(packet + sizeof(struct ethhdr) + sizeof(struct ipv4hdr), data_length);

	udp->source_port = sys_cpu_to_be16(source_port);
	udp->dest_port = sys_cpu_to_be16(target_port);
	udp->length = sys_cpu_to_be16(sizeof(struct udphdr) + data_length);
	udp->checksum = 0;  /* UDP checksum is optional for IPv4 */

	return sizeof(struct ethhdr) + sizeof(struct ipv4hdr) + sizeof(struct udphdr) + data_length;
}

int send_udp_packet(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
		     const uint8_t my_ip_addr[IP_LEN], const uint8_t target_mac_addr[ETH_ALEN],
			 const uint8_t target_ip_addr[IP_LEN], uint16_t source_port, uint16_t target_port, const uint8_t *data, uint16_t length)
{
	uint8_t packet[sizeof(struct ethhdr) + sizeof(struct ipv4hdr) + sizeof(struct udphdr) + length];
	uint16_t packet_length = make_udp_packet(packet, my_mac_addr, my_ip_addr, target_mac_addr, target_ip_addr, source_port, target_port, data, length);

	return send_packet(spi, packet, packet_length);
}

int receive_udp_packet(const struct spi_dt_spec *spi, uint8_t source_mac_addr[ETH_ALEN], uint16_t *source_port, uint8_t *data, uint16_t *length)
{
	uint8_t packet[MAX_PACKET_SIZE] = {
		0,
	};

	*length = 0;

	while (*length == 0) {
		if (receive_packet(spi, packet, length) < 0) {
			printk("Failed to receive UDP packet\n");
			return -1;
		}
	}

	struct ethhdr *eth = (struct ethhdr *)packet;
	struct ipv4hdr *ipv4 = (struct ipv4hdr *)(eth + 1);
	struct udphdr *udp = (struct udphdr *)(ipv4 + 1);

	if (eth->h_proto != sys_cpu_to_be16(ETH_P_IP)) {
		printk("Invalid IP packet: 0x%04x\n", eth->h_proto);
	}

	if (ipv4->proto != IPPROTO_UDP) {
		printk("Invalid UDP packet: 0x%04x\n", ipv4->proto);
		return -1;
	}

	memcpy(source_mac_addr, eth->h_source, ETH_ALEN);
	*source_port = sys_be16_to_cpu(udp->source_port);
	*length = sys_be16_to_cpu(udp->length) - sizeof(struct udphdr);
	memcpy(data, (uint8_t *)udp + sizeof(struct udphdr), *length);

	printk("UDP Packet Received from %02x:%02x:%02x:%02x:%02x:%02x (%d.%d.%d.%d:%d)\n", eth->h_source[0], eth->h_source[1], eth->h_source[2],
	       eth->h_source[3], eth->h_source[4], eth->h_source[5], (ipv4->src >> 24) & 0xFF, (ipv4->src >> 16) & 0xFF, (ipv4->src >> 8) & 0xFF, ipv4->src & 0xFF, *source_port);

	return 0;
}
