#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "Command.h"

#define ROBOT_ARM_COMMAND_INTERVAL_NS (2 * NSEC_PER_SEC)

static struct tt_Node node;
static struct tt_Publisher pub;

static void move_joint(uint8_t joint, int32_t angle) {
    struct CommandData command = {
        .joint = joint,
        .angle = angle,
    };
    int32_t ret = tt_Publisher_publish(&pub, (struct tt_Data*)&command);
    if (ret != 0) {
        printk("Failed to publish command: %d\n", ret);
    }
}

static void initial_state(void) {
    for (uint8_t i = 1; i <= 6; i++) {
        move_joint(i, 0);
    }
    k_msleep(2000);
    move_joint(1, 45);
    move_joint(2, 20);
    move_joint(3, 10);
    k_msleep(1000);
}

static void move_command(struct tt_Node* node, uint64_t time, void* param) {
    int64_t idx = (int64_t)param;

    if (idx == 0) {
        move_joint(1, 25);
        move_joint(3, 25);
    } else {
        move_joint(1, 65);
        move_joint(3, 5);
    }

    tt_Node_schedule(node, time, move_command, (void*)(1 - idx));
}

int main(void)
{
    printk("Master Node Started\n");

    _tt_CONFIG.broadcast = "192.168.10.255";

    int32_t ret = tt_Node_create(&node);
    if (ret != 0) {
        printk("Failed to create node: %d\n", ret);
        return ret;
    }

    printk("Master Node Created, Node ID: %d\n", node.id);

    ret = tt_Node_create_publisher(&node, &pub, &CommandTopic, "command_topic");
    if (ret != 0) {
        printk("Failed to create publisher: %d\n", ret);
        return ret;
    }

    k_msleep(5000);  /* Wait for calibration,  TODO: Use services to check if calibration is done */

    initial_state();

    tt_Node_schedule(&node, ROBOT_ARM_COMMAND_INTERVAL_NS, move_command, (void*)0);

    tt_Node_poll(&node);

    tt_Node_destroy(&node);

	return 0;
}
