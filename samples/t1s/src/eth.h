#ifndef ETH_H
#define ETH_H

#include <stdint.h>

/* FIXME: These are borrowed from linux headers since network features are not available */
/* Remove this file and use the one included in Zephyr after the issue's fixed */

#define IP_LEN (4)
#define PACKET_SIZE_ARP (sizeof(struct ethhdr) + sizeof(struct arphdr_ipv4))

#define ETH_ALEN 6
#define ETH_HLEN 14
#define ARP_HLEN 28

#define ETH_P_IP	0x0800		/* Internet Protocol packet	*/
#define ETH_P_ARP	0x0806		/* Address Resolution packet	*/

#define	ARPOP_REQUEST	1		/* ARP request.  */
#define	ARPOP_REPLY	2		/* ARP reply.  */

#define PF_INET		2	/* IP protocol family.  */
#define AF_INET PF_INET

struct ethhdr {
	unsigned char	h_dest[ETH_ALEN];	/* destination eth addr	*/
	unsigned char	h_source[ETH_ALEN];	/* source ether addr	*/
	uint16_t		h_proto;		/* packet type ID field	*/
} __attribute__((packed));

struct ethhdr_l {
    unsigned char h_dest[ETH_ALEN];
    unsigned char h_source[ETH_ALEN];
    unsigned short h_proto;
};

struct arphdr_l {
    unsigned short ar_hrd;
    unsigned short ar_pro;
    unsigned char ar_hln;
    unsigned char ar_pln;
    unsigned short ar_op;
    unsigned char ar_sha[ETH_ALEN];
    unsigned char ar_sip[4];
    unsigned char ar_tha[ETH_ALEN];
    unsigned char ar_tip[4];
};

struct arphdr
  {
    unsigned short int ar_hrd;		/* Format of hardware address.  */
    unsigned short int ar_pro;		/* Format of protocol address.  */
    unsigned char ar_hln;		/* Length of hardware address.  */
    unsigned char ar_pln;		/* Length of protocol address.  */
    unsigned short int ar_op;		/* ARP opcode (command).  */
  };

struct arphdr_ipv4 {
    struct arphdr arp;
    uint8_t sender_mac[ETH_ALEN];
    uint8_t sender_ip[IP_LEN];
    uint8_t target_mac[ETH_ALEN];
    uint8_t target_ip[IP_LEN];
};

// ARP table entry regarding IPv4
struct arp_entry {
    uint8_t ip[IP_LEN];
    uint8_t mac[ETH_ALEN];
};

#endif /* ETH_H */