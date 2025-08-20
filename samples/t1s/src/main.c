#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#include "lan8651.h"
#include "arp.h"

#define SPI_NODE DT_ALIAS(spi0)

struct spi_dt_spec spi_dev;

const uint8_t my_mac_addr[ETH_ALEN] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, };
const uint8_t my_ip_addr[IP_LEN] = {10, 1, 1, 2, };
const uint8_t target_ip_addr[IP_LEN] = {10, 1, 1, 1, };


void arp_test() {
    send_arp_request(&spi_dev, my_mac_addr, my_ip_addr, target_ip_addr);
    receive_arp_reply(&spi_dev);
}

int main(void)
{
    spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
    spi_dev.config.frequency = 25000000;
    spi_dev.config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;

    set_register(&spi_dev, PLCA_MODE_COORDINATOR);

    arp_test();

    return 0;
}