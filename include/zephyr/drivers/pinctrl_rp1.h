#ifndef ZEPHYR_DRIVERS_PINCTRL_RP1_H_
#define ZEPHYR_DRIVERS_PINCTRL_RP1_H_

#include <zephyr/device.h>
#include <stdint.h>

struct rp1_pinctrl_api {
    void (*set_alt)(const struct device *dev,
                    uint32_t pin,
                    uint32_t func);

    uint32_t (*get_ctrl)(const struct device *dev,
                         uint32_t pin);
};

#endif
