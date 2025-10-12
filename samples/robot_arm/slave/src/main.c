#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <zephyr/drivers/gpio.h>

#include <tickle/tickle.h>
#include "Command.h"

#define PULSE_COUNT 20

#define JOINT_COUNT 3

#define JOINTS_NODE DT_ALIAS(joints0)
#define BUTTONS_NODE DT_ALIAS(buttons0)

static inline bool is_my_joint(uint8_t joint) {
#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
    return joint == 1 || joint == 2 || joint == 3;
#else
    return joint == 4 || joint == 5 || joint == 6;
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
}

enum JOINT_DIRECTION {
    LEFT,
    RIGHT,
};

static const struct gpio_dt_spec joint1_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 0);
static const struct gpio_dt_spec joint1_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 1);
static const struct gpio_dt_spec joint1_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 0);
static const struct gpio_dt_spec joint2_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 2);
static const struct gpio_dt_spec joint2_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 3);
static const struct gpio_dt_spec joint2_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 1);
static const struct gpio_dt_spec joint3_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 4);
static const struct gpio_dt_spec joint3_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 5);
static const struct gpio_dt_spec joint3_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 2);

static const struct gpio_dt_spec dummy_dt_spec = {
    .port = NULL,
    .pin = 0,
    .dt_flags = 0,
};

static const struct gpio_dt_spec pulse_pins[] = {
    dummy_dt_spec,
#ifndef CONFIG_ROBOTARM_SLAVE_IS_FIRST
    dummy_dt_spec,
    dummy_dt_spec,
    dummy_dt_spec,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
    joint1_pulse,
    joint2_pulse,
    joint3_pulse,
};

static const struct gpio_dt_spec direction_pins[] = {
    dummy_dt_spec,
#ifndef CONFIG_ROBOTARM_SLAVE_IS_FIRST
    dummy_dt_spec,
    dummy_dt_spec,
    dummy_dt_spec,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
    joint1_direction,
    joint2_direction,
    joint3_direction,
};

static const struct gpio_dt_spec limit_buttons[] = {
    dummy_dt_spec,
#ifndef CONFIG_ROBOTARM_SLAVE_IS_FIRST
    dummy_dt_spec,
    dummy_dt_spec,
    dummy_dt_spec,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
    joint1_limit_button,
    joint2_limit_button,
    joint3_limit_button,
};

static bool check_joint_limit_button(uint8_t joint) {
    if (!is_my_joint(joint)) {
        printk("Error: Joint %d is out of range\n", joint);
        return false;
    }

    return gpio_pin_get_dt(&limit_buttons[joint]) != 0;
}

static void move_joint_step(uint8_t joint, enum JOINT_DIRECTION direction) {
    // TODO: Check limit button
    switch (direction) {
        case LEFT:
            gpio_pin_set_dt(&direction_pins[joint], 0);
            break;
        case RIGHT:
            gpio_pin_set_dt(&direction_pins[joint], 1);
            break;
    }

    for (int i = 0; i < PULSE_COUNT; i++) {
        gpio_pin_set_dt(&pulse_pins[joint], 0);
        k_busy_wait(100);
        gpio_pin_set_dt(&pulse_pins[joint], 1);
        k_busy_wait((joint == 1 || joint == 3) ? 250 : 500);
    }
}

static void move_joint(uint8_t joint, int32_t angle) {
    // TODO: Find out how many steps to move
    move_joint_step(joint, angle > 0 ? RIGHT : LEFT);
}

static void do_calibration(void) {
    printk("Starting calibration\n");

#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
    while (true) {
        move_joint_step(1, LEFT);
        if (check_joint_limit_button(1)) {
            break;
        }
    }

    while (true) {
        move_joint_step(2, LEFT);
        if (check_joint_limit_button(2)) {
            break;
        }
    }

    while (true) {
        move_joint_step(3, RIGHT);
        if (check_joint_limit_button(3)) {
            break;
        }
    }

    for (int i = 0; i < 50; i++) {
        move_joint_step(3, LEFT);
    }
#else
    while (true) {
        move_joint_step(4, LEFT);
        if (check_joint_limit_button(4)) {
            break;
        }
    }
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
}

static void command_callback(struct tt_Subscriber* sub, uint64_t timestamp, uint16_t seq_no, struct CommandData* data) {
    printk("Received command: %u, %d\n", data->joint, data->angle);
    if (is_my_joint(data->joint)) {
        move_joint(data->joint, data->angle);
    }
}

int main(void)
{
    printk("Slave Node Started\n");

    _tt_CONFIG.addr = CONFIG_NET_CONFIG_MY_IPV4_ADDR;
    _tt_CONFIG.broadcast = "192.168.10.255";

    do_calibration();

    struct tt_Node node;
    int32_t ret = tt_Node_create(&node);
    if (ret != 0) {
        printk("Failed to create node: %d\n", ret);
        return ret;
    }

    printk("Slave Node Created, Node ID: %d\n", node.id);

    struct tt_Subscriber sub;

    ret = tt_Node_create_subscriber(&node, &sub, &CommandTopic, "command_topic", (tt_SUBSCRIBER_CALLBACK)command_callback);
    if (ret != 0) {
        printk("Failed to create subscriber: %d\n", ret);
        return ret;
    }

    tt_Node_poll(&node);

    tt_Node_destroy(&node);

	return 0;
}
