#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <zephyr/drivers/gpio.h>

#include <tickle/tickle.h>
#include "Command.h"
#include "State.h"

#define ROBOT_ARM_STATE_INTERVAL_NS (100ULL * NSEC_PER_MSEC)

#define PPR 16000

#define PULSE_COUNT 2
#define JOINT_COUNT 3

#define JOINTS_NODE  DT_ALIAS(joints0)
#define BUTTONS_NODE DT_ALIAS(buttons0)

static inline bool is_my_joint(uint8_t joint)
{
#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
	return joint == 1 || joint == 3 || joint == 5;
#else
	return joint == 2 || joint == 4 || joint == 6;
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
}

static int DELAY_MULTIPLIER = 1;

enum JOINT_DIRECTION {
	LEFT,
	RIGHT,
};

static const struct gpio_dt_spec joint1_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 0);
static const struct gpio_dt_spec joint1_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 1);
static const struct gpio_dt_spec joint1_limit_button =
	GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 0);
static const struct gpio_dt_spec joint2_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 2);
static const struct gpio_dt_spec joint2_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 3);
static const struct gpio_dt_spec joint2_limit_button =
	GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 1);
static const struct gpio_dt_spec joint3_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 4);
static const struct gpio_dt_spec joint3_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 5);
static const struct gpio_dt_spec joint3_limit_button =
	GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 2);

static const struct gpio_dt_spec dummy_dt_spec = {
	.port = NULL,
	.pin = 0,
	.dt_flags = 0,
};

static const struct gpio_dt_spec pulse_pins[] = {
	dummy_dt_spec,
#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
	joint1_pulse,  dummy_dt_spec, joint2_pulse, dummy_dt_spec, joint3_pulse, dummy_dt_spec,
#else
	dummy_dt_spec, joint1_pulse, dummy_dt_spec, joint2_pulse, dummy_dt_spec, joint3_pulse,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
};

static const struct gpio_dt_spec direction_pins[] = {
	dummy_dt_spec,
#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
	joint1_direction, dummy_dt_spec,    joint2_direction,
	dummy_dt_spec,    joint3_direction, dummy_dt_spec,
#else
	dummy_dt_spec,    joint1_direction, dummy_dt_spec,
	joint2_direction, dummy_dt_spec,    joint3_direction,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
};

static const struct gpio_dt_spec limit_buttons[] = {
	dummy_dt_spec,
#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
	joint1_limit_button, dummy_dt_spec,       joint2_limit_button,
	dummy_dt_spec,       joint3_limit_button, dummy_dt_spec,
#else
	dummy_dt_spec,       joint1_limit_button, dummy_dt_spec,
	joint2_limit_button, dummy_dt_spec,       joint3_limit_button,
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
};

static const uint32_t speeds[7] = {
	0, 5000, 1500, 4000, 2500, 2500, 2500,
};

struct tt_Publisher pub;

static int32_t current_angles[7] = {
	0,
};

static bool check_joint_limit_button(uint8_t joint)
{
	if (!is_my_joint(joint)) {
		printk("Error: Joint %d is out of range\n", joint);
		return false;
	}

	return gpio_pin_get_dt(&limit_buttons[joint]) != 0;
}

static void move_joint_step(uint8_t joint, enum JOINT_DIRECTION direction)
{
	switch (direction) {
	case LEFT:
		if (joint == 1 || joint == 2 || joint == 4) {
			if (check_joint_limit_button(joint)) {
				printk("Joint %d limit button pressed\n", joint);
				break;
			}
		}
		gpio_pin_set_dt(&direction_pins[joint], 0);
		break;
	case RIGHT:
		if (joint == 3) {
			if (check_joint_limit_button(joint)) {
				printk("Joint %d limit button pressed\n", joint);
				break;
			}
		}
		gpio_pin_set_dt(&direction_pins[joint], 1);
		break;
	}

	for (int i = 0; i < PULSE_COUNT; i++) {
		gpio_pin_set_dt(&pulse_pins[joint], 0);
		k_busy_wait(((USEC_PER_SEC / speeds[joint]) / 2) * DELAY_MULTIPLIER);
		gpio_pin_set_dt(&pulse_pins[joint], 1);
		k_busy_wait(((USEC_PER_SEC / speeds[joint]) / 2) * DELAY_MULTIPLIER);
		gpio_pin_set_dt(&pulse_pins[joint], 0);
		k_busy_wait(((USEC_PER_SEC / speeds[joint]) / 2) * DELAY_MULTIPLIER);
		gpio_pin_set_dt(&pulse_pins[joint], 1);
		k_busy_wait(((USEC_PER_SEC / speeds[joint]) / 2) * DELAY_MULTIPLIER);
	}
}

static void move_joint(uint8_t joint, int32_t angle)
{
	int32_t angle_diff = angle - current_angles[joint];
	if (joint == 3) {
		angle_diff *= -1;
	}

	int32_t steps = (angle_diff * PPR) / 360;
	if (steps < 0) {
		steps *= -1;
	}

	for (int32_t i = 0; i < steps; i++) {
		move_joint_step(joint, angle_diff > 0 ? RIGHT : LEFT);
	}

	current_angles[joint] = angle;
	struct StateData state = {
		.id = joint,
		.status = angle,
	};

	/* Publish the state after the movement */
	/* Angles during the movement do not matter */
	/* It's either done or not */
	int32_t ret = tt_Publisher_publish(&pub, (struct tt_Data *)&state);
	if (ret != 0) {
		printk("Failed to publish state: %d\n", ret);
	}
}

static void do_calibration(void)
{
	printk("Starting calibration\n");
	DELAY_MULTIPLIER = 2; /* Calibrate slower to be safe */

#ifdef CONFIG_ROBOTARM_SLAVE_IS_FIRST
	printk("Calibrating joint 1\n");
	while (true) {
		move_joint_step(1, LEFT);
		if (check_joint_limit_button(1)) {
			break;
		}
	}

	printk("Calibrating joint 3\n");
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
	printk("Calibrating joint 2\n");
	while (true) {
		move_joint_step(2, LEFT);
		if (check_joint_limit_button(2)) {
			break;
		}
	}

	for (int i = 0; i < 50; i++) {
		move_joint_step(2, RIGHT);
	}

	/* Joint 4 is not used for now */

	/*
	while (true) {
	    move_joint_step(4, LEFT);
	    if (check_joint_limit_button(4)) {
		break;
	    }
	}
	*/
#endif /* CONFIG_ROBOTARM_SLAVE_IS_FIRST */
	DELAY_MULTIPLIER = 1;
	printk("Calibration finished\n");
}

static void command_callback(struct tt_Subscriber *sub, uint64_t timestamp, uint16_t seq_no,
			     struct CommandData *data)
{
	if (is_my_joint(data->joint) && data->angle != current_angles[data->joint]) {
		move_joint(data->joint, data->angle);
	}
}

static void announce_state(struct tt_Node *node, uint64_t time, void *param)
{
	/* Announce the state of the joints */
	struct StateData state;
	int32_t ret;

	for (int i = 1; i <= 6; i++) {
		if (!is_my_joint(i)) {
			continue;
		}
		state.id = i;
		state.status = current_angles[i];

		ret = tt_Publisher_publish(&pub, (struct tt_Data *)&state);
		if (ret != 0) {
			printk("Failed to publish state: %d\n", ret);
		}
	}
	tt_Node_schedule(node, time + ROBOT_ARM_STATE_INTERVAL_NS, announce_state, NULL);
}

int main(void)
{
	int32_t ret;

	printk("Slave Node Started\n");

	ret = gpio_pin_configure_dt(&joint1_pulse, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint1_pulse: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint1_direction, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint1_direction: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint1_limit_button, GPIO_INPUT);
	if (ret != 0) {
		printk("Failed to configure joint1_limit_button: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint2_pulse, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint2_pulse: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint2_direction, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint2_direction: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint2_limit_button, GPIO_INPUT);
	if (ret != 0) {
		printk("Failed to configure joint2_limit_button: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint3_pulse, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint3_pulse: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint3_direction, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Failed to configure joint3_direction: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&joint3_limit_button, GPIO_INPUT);
	if (ret != 0) {
		printk("Failed to configure joint3_limit_button: %d\n", ret);
		return ret;
	}

	_tt_CONFIG.broadcast = "192.168.10.255";

	do_calibration();

	struct tt_Node node;
	ret = tt_Node_create(&node);
	if (ret != 0) {
		printk("Failed to create node: %d\n", ret);
		return ret;
	}

	printk("Slave Node Created, Node ID: %d\n", node.id);

	struct tt_Subscriber sub;

	ret = tt_Node_create_subscriber(&node, &sub, &CommandTopic, "command_topic",
					(tt_SUBSCRIBER_CALLBACK)command_callback);
	if (ret != 0) {
		printk("Failed to create subscriber: %d\n", ret);
		return ret;
	}

	ret = tt_Node_create_publisher(&node, &pub, &StateTopic, "state_topic");
	if (ret != 0) {
		printk("Failed to create publisher: %d\n", ret);
		return ret;
	}

	tt_Node_schedule(&node, tt_get_ns() + ROBOT_ARM_STATE_INTERVAL_NS, announce_state, NULL);

	tt_Node_poll(&node);

	tt_Node_destroy(&node);

	return 0;
}
