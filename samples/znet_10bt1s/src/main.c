#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>

#include <zephyr/drivers/ethernet/eth_lan865x.h>

#define TX_INTERVAL_MS 1000

static void rx_callback(const uint8_t *data, size_t len, void *user_data)
{
    ARG_UNUSED(user_data);

    printk("RX fragment len=%u\n", len);

    for (size_t i = 0; i < len; i++) {

        printk("%02x ", data[i]);

        if ((i & 0x0f) == 0x0f)
            printk("\n");
    }

    printk("\n");
}

static void build_test_frame(uint8_t *frame, uint16_t counter)
{
    memset(frame, 0, 60);

    /* destination = broadcast */
    frame[0] = 0xff;
    frame[1] = 0xff;
    frame[2] = 0xff;
    frame[3] = 0xff;
    frame[4] = 0xff;
    frame[5] = 0xff;

    /* source MAC (example) */
    frame[6]  = 0xd0;
    frame[7]  = 0xd1;
    frame[8]  = 0x95;
    frame[9]  = 0x30;
    frame[10] = 0x23;
    frame[11] = 0x00;

    /* Ethertype */
    frame[12] = 0x88;
    frame[13] = 0xb5;

    /* payload test pattern */
    frame[14] = 0x86;
    frame[15] = 0x51;

    frame[16] = counter >> 8;
    frame[17] = counter & 0xff;
}

int main(void)
{
    const struct device *dev;
    struct net_if *iface;

    uint8_t frame[60];
    uint16_t counter = 0;

    printk("LAN865x raw broadcast TX/RX test\n");

    dev = DEVICE_DT_GET(DT_NODELABEL(lan865x));

    if (!device_is_ready(dev)) {
        printk("LAN865x device not ready\n");
        return 0;
    }

    iface = net_if_get_default();

    if (!iface) {
        printk("No network interface\n");
        return 0;
    }

    printk("Register RX callback\n");

    lan865x_register_rx_callback(dev, rx_callback, NULL);

    printk("Start TX loop\n");

    while (1) {

        build_test_frame(frame, counter);

        int ret = lan865x_tx_frame(dev, frame, sizeof(frame));

        printk("broadcast TX counter=%u ret=%d\n", counter, ret);

        counter++;

        k_sleep(K_MSEC(TX_INTERVAL_MS));
    }
}