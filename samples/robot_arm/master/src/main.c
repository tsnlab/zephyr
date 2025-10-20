#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "Command.h"

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

    int32_t angles[2][7] = {
        {0, 25, 20, 25, 0, 0, 0},
        {0, 65, 20, 5, 0, 0, 0},
    };

    while (true) {
        for (int i = 0; i < 2; i++) {  /* Actions */
            for (int j = 1; j <= 6; j++) {  /* Joints */
                move_joint(j, angles[i][j]);
            }
            k_msleep(2000);
        }
    }

    tt_Node_poll(&node);

    tt_Node_destroy(&node);

	return 0;
}
