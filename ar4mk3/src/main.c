/*
 * Copyright (c) 2025 TSN Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#define LOW		0
#define HIGH		1
#define SLEEP_TIME_MS	1000

/*
 *	GPIO Pin Mapping
 *
 * Output
 * GPIOB 2  ->  LED
 * GPIOA 0  ->  Stepping
 * GPIOA 1  ->  Direction
 *
 * Input
 * GPIOA 29 <-  Rotate CCW (LOW to GPIOA 1)
 * GPIOA 28 <-  Rotate CW  (HIGH to GPIOA 1)
 * GPIOB 3  <-  Stop Button
 */

#define BUTTONS_NODE	DT_ALIAS(buttons0)
#define LED0_NODE	DT_ALIAS(led0)
#define STEPPER_NODE	DT_ALIAS(stepper0)

static const struct gpio_dt_spec led_dt = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct gpio_dt_spec estop_dt = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 0);
static const struct gpio_dt_spec rotate_ccw_dt = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 1);
static const struct gpio_dt_spec rotate_cw_dt = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 2);

static const struct gpio_dt_spec motor_step_dt = GPIO_DT_SPEC_GET_BY_IDX(STEPPER_NODE, gpios, 0);
static const struct gpio_dt_spec motor_dir_dt = GPIO_DT_SPEC_GET_BY_IDX(STEPPER_NODE, gpios, 1);

static int init_gpio(const struct gpio_dt_spec* gpio, gpio_flags_t flags);
static int initialize(void);

static inline void busy_wait_ms(unsigned int ms)
{
	k_busy_wait(ms * SLEEP_TIME_MS);
}

int init_gpio(const struct gpio_dt_spec* gpio, gpio_flags_t flags)
{
	int ret;

	if (!gpio_is_ready_dt(gpio)) {
		printk("Error: GPIO %s is not ready\n",
		       gpio->port->name);
		return -1;
	}

	ret = gpio_pin_configure_dt(gpio, flags);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, gpio->port->name, gpio->pin);
		return -1;
	} else {
		printk("Set up GPIO at %s pin %d\n", gpio->port->name, gpio->pin);
	}
	return 0;
}

#define CHECK_INIT_GPIO(ret) \
	if (ret < 0) { \
		return -1; \
	}

int initialize(void)
{
	CHECK_INIT_GPIO(init_gpio(&estop_dt, GPIO_INPUT));
	CHECK_INIT_GPIO(init_gpio(&rotate_ccw_dt, GPIO_INPUT));
	CHECK_INIT_GPIO(init_gpio(&rotate_cw_dt, GPIO_INPUT));
	CHECK_INIT_GPIO(init_gpio(&led_dt, GPIO_OUTPUT));
	CHECK_INIT_GPIO(init_gpio(&motor_step_dt, GPIO_OUTPUT));
	CHECK_INIT_GPIO(init_gpio(&motor_dir_dt, GPIO_OUTPUT));
	return 0;
}

int main(void)
{
	int estop;
	int motor;
	int prev_motor;
	int dir;

	if (initialize() < 0) {
		return 0;
	}
	printk("Initialization done\n");
	motor = 0;
	while (1) {
		prev_motor = motor;
		estop = gpio_pin_get_dt(&estop_dt);
		dir = gpio_pin_get_dt(&rotate_ccw_dt) > 0 ? 1 : 0;
		motor = dir ^ (gpio_pin_get_dt(&rotate_cw_dt) > 0 ? 1 : 0);
		printk("motor=%d  dir=%d\n", motor, dir);
		if (estop > 0) {
			motor = 0;
			gpio_pin_set_dt(&led_dt, HIGH);
			printk("estop\n");
			busy_wait_ms(2);
			continue;
		} else {
			gpio_pin_set_dt(&led_dt, LOW);
		}
		if (motor == 0) {
			busy_wait_ms(2);
			continue;
		}
		if (prev_motor == 0) {
			gpio_pin_set_dt(&motor_dir_dt, dir);
			k_busy_wait(15);
		}
		gpio_pin_set_dt(&motor_step_dt, 0);
		busy_wait_ms(2);
		gpio_pin_set_dt(&motor_step_dt, 1);
	}
	return 0;
}
