#ifndef ARP_H
#define ARP_H

#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "eth.h"

int send_arp_request(const struct spi_dt_spec* spi, const uint8_t my_mac_addr[ETH_ALEN], const uint8_t my_ip_addr[IP_LEN], const uint8_t target_ip_addr[IP_LEN]);
int receive_arp_reply(const struct spi_dt_spec* spi);


#endif  /* ARP_H */
