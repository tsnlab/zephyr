#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "Command.h"
#include "State.h"

#define PING_INTERVAL_NS (1000ULL * NSEC_PER_USEC)

static struct tt_Node node;
static struct tt_Publisher pub;
static struct tt_Subscriber sub;

static void send_ping(struct tt_Node *node, uint64_t time, void *param)
{
	printk("Sending ping\n");
	tt_Node_schedule(node, time + PING_INTERVAL_NS, send_ping, NULL);
}

static void pong_callback(struct tt_Subscriber *sub, uint64_t timestamp, uint16_t seq_no, struct StateData *data)
{
	/* TODO: print timestamp */
	printk("Pong received\n");
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

	// ret = tt_Node_create_publisher(&node, &pub, &CommandTopic, "command_topic");
	// if (ret != 0) {
	// 	printk("Failed to create publisher: %d\n", ret);
	// 	return ret;
	// }

	// ret = tt_Node_create_subscriber(&node, &sub, &StateTopic, "state_topic",
	// 				(tt_SUBSCRIBER_CALLBACK)state_callback);
	// if (ret != 0) {
	// 	printk("Failed to create subscriber: %d\n", ret);
	// 	return ret;
	// }

	// tt_Node_schedule(&node, tt_get_ns() + PING_INTERVAL_NS, send_ping, NULL);

	tt_Node_poll(&node);

	tt_Node_destroy(&node);

	return 0;
}
