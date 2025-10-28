#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "Command.h"
#include "State.h"

#define ROBOT_ARM_COMMAND_INTERVAL_NS (1000ULL * NSEC_PER_USEC)

static struct tt_Node node;
static struct tt_Publisher pub;
static struct tt_Subscriber sub;

static int32_t states[7] = {-1, -1, -1, -1, -1, -1, -1};
static int32_t desired[7] = {0, 0, 0, 0, 0, 0, 0};

#define MOVE_COUNT 18

static int32_t moves[MOVE_COUNT][2] = {
    {1, 45},
    {2, 15},
    {3, 50},
    {3, 80},
    {3, 50},
    {3, 80},
    {3, 50},
    {3, 80},
    {3, 50},
    /* return */
    {1, 135},
    {2, 25},
    {3, 80},
    {3, 50},
    {3, 80},
    {3, 50},
    {3, 80},
    {3, 50},
    {3, 80},
};

static inline bool is_connected() {
    for (uint8_t i = 1; i <= 6; i++) {
        if (states[i] < 0) {
            return false;
        }
    }
    return true;
}

static inline bool is_ok() {
    for (uint8_t i = 1; i <= 6; i++) {
        if (states[i] != desired[i]) {
            return false;
        }
    }
    return true;
}

static void move_joint(uint8_t joint, int32_t angle) {
    desired[joint] = angle;
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
    move_joint(1, 90);
    move_joint(2, 20);
    move_joint(3, 10);
}

static inline int64_t next_idx(int64_t idx) {
    idx++;
    if (idx == MOVE_COUNT) {
        idx = 0;
    }
    return idx;
}

static void move_command(struct tt_Node* node, uint64_t time, void* param) {
    int64_t idx = (int64_t)param;

    if (is_ok()) {
        move_joint(moves[idx][0], moves[idx][1]);
        idx = next_idx(idx);
    } else {
        for (int i = 1; i <= 3; i++) {
            if (states[i] != desired[i]) {
                move_joint(i, desired[i]);
            }
        }
    }

    tt_Node_schedule(node, time + ROBOT_ARM_COMMAND_INTERVAL_NS, move_command, (void*)idx);
}

static void initialize(struct tt_Node* node, uint64_t time, void* param) {
    if (is_connected()) {
        initial_state();
        tt_Node_schedule(node, time + ROBOT_ARM_COMMAND_INTERVAL_NS, move_command, (void*)0);
    } else {
        tt_Node_schedule(node, time + ROBOT_ARM_COMMAND_INTERVAL_NS, initialize, NULL);
    }
}

static void state_callback(struct tt_Subscriber* sub, uint64_t timestamp, uint16_t seq_no, struct StateData* data) {
    if (data->id >= 1 && data->id <= 6) {
        states[data->id] = data->status;
    }
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

    ret = tt_Node_create_subscriber(&node, &sub, &StateTopic, "state_topic", (tt_SUBSCRIBER_CALLBACK)state_callback);
    if (ret != 0) {
        printk("Failed to create subscriber: %d\n", ret);
        return ret;
    }

    tt_Node_schedule(&node, tt_get_ns() + ROBOT_ARM_COMMAND_INTERVAL_NS, initialize, NULL);

    tt_Node_poll(&node);

    tt_Node_destroy(&node);

	return 0;
}
