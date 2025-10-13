#ifndef ETH_H
#define ETH_H

#include <stdint.h>

/* FIXME: These are borrowed from linux headers since network features are not available */
/* Remove this file and use the one included in Zephyr after the issue's fixed */

#define IP_LEN          (4)
#define PACKET_SIZE_ARP (sizeof(struct ethhdr) + sizeof(struct arphdr_ipv4))

#define ETH_ALEN 6
#define ETH_HLEN 14

#define ETH_P_IP  0x0800 /* Internet Protocol packet	*/

#define PF_INET 2 /* IP protocol family.  */
#define AF_INET PF_INET

struct ethhdr {
	unsigned char h_dest[ETH_ALEN];   /* destination eth addr	*/
	unsigned char h_source[ETH_ALEN]; /* source ether addr	*/
	uint16_t h_proto;                 /* packet type ID field	*/
} __attribute__((packed));

struct ethhdr_l {
	unsigned char h_dest[ETH_ALEN];
	unsigned char h_source[ETH_ALEN];
	unsigned short h_proto;
};

#endif /* ETH_H */
