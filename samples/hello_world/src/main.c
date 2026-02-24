#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/pinctrl_rp1.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define RP1_NODE DT_NODELABEL(rp1_pinctrl)
#define TEST_PIN 17

int main(void)
{
    const struct device *dev;
    const struct rp1_pinctrl_api *api;

    printk("\n=== RP1 PINCTRL MODE CYCLING TEST ===\n");

    dev = DEVICE_DT_GET(RP1_NODE);

    if (!device_is_ready(dev)) {
        printk("Device not ready\n");
        return 0;
    }

    api = (const struct rp1_pinctrl_api *)dev->api;

    uint32_t modes[] = {0,1,2,3,4,5};
    int count = ARRAY_SIZE(modes);

    while (1) {
        for (int i = 0; i < count; i++) {

            printk("Setting GPIO%d -> ALT%d\n",
                   TEST_PIN, modes[i]);

            api->set_alt(dev, TEST_PIN, modes[i]);

            uint32_t val =
                api->get_ctrl(dev, TEST_PIN);

            printk("CTRL = 0x%08x\n", val);

            k_sleep(K_SECONDS(1));
        }
    }
	return 0;
}
