/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	int count;
	int repeat = 0;
	bool led_state = true;
	uint32_t curr_cycle;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printk("LED state: %3s, repeat: %d\n", led_state ? "ON" : "OFF", repeat);
		// k_msleep(SLEEP_TIME_MS);
		count = 0;
		while (count < 600) {
			count++;
			curr_cycle = sys_read32(0xA0F2A000 + 0x014UL);
			printk("\rtimer count value: %4d", curr_cycle);
		}
		printk("\n");
		repeat++;
	}
	return 0;
}
