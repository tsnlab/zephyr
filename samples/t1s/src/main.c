#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#include "eth.h"
#include "lan8651.h"
#include "arp.h"
#include "perf.h"

#define THROUGHPUT_DURATION 10
#define THROUGHPUT_WARMUP 0
#define THROUGHPUT_PAYLOAD_SIZE 1500

#define SPI_NODE DT_ALIAS(spi0)

struct spi_dt_spec spi_dev;

const uint8_t my_mac_addr[ETH_ALEN] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, };
const uint8_t target_mac_addr[ETH_ALEN] = {0xd0, 0xd1, 0x95, 0x30, 0x23, 0x01};
const uint8_t my_ip_addr[IP_LEN] = {10, 1, 1, 2, };
const uint8_t target_ip_addr[IP_LEN] = {10, 1, 1, 1, };


static void arp_test() {
    send_arp_request(&spi_dev, my_mac_addr, my_ip_addr, target_ip_addr);
    receive_arp_reply(&spi_dev);
}

static void throughput_test() {
    uint8_t payload[MAX_PERF_PAYLOAD] = {0};
    uint16_t len = 0;
    send_perf_req_packet(&spi_dev, my_mac_addr, target_mac_addr, THROUGHPUT_DURATION, THROUGHPUT_WARMUP);
    len = make_perf_data_packet(my_mac_addr, target_mac_addr, payload, THROUGHPUT_PAYLOAD_SIZE);
    struct ethhdr* eth = (struct ethhdr*)payload;
    struct perf_header* perf = (struct perf_header*)(eth + 1);
    uint32_t i = 0;
    while (true) {
        perf->id = sys_cpu_to_be32(i);
        send_perf_data_packet(&spi_dev, payload, len);
        i++;
    }
}

static void latency_test() {
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

int main(void)
{
    spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
    spi_dev.config.frequency = 25000000;
    spi_dev.config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;

    set_register(&spi_dev, PLCA_MODE_COORDINATOR);

    // arp_test();
    // throughput_test();
    latency_test();

    return 0;
}