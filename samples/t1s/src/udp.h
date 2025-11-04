#ifndef UDP_H
#define UDP_H

#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "eth.h"

int send_udp_packet(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
		     const uint8_t my_ip_addr[IP_LEN], const uint8_t target_mac_addr[ETH_ALEN],
			 const uint8_t target_ip_addr[IP_LEN], uint16_t source_port, uint16_t target_port, const uint8_t *data, uint16_t length);
int receive_udp_packet(const struct spi_dt_spec *spi, uint8_t source_mac_addr[ETH_ALEN], uint16_t *source_port, uint8_t *data, uint16_t *length);

#endif /* UDP_H */
