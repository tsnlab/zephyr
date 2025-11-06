#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "type/Perf.h"

#define PING_INTERVAL_NS (1000ULL * NSEC_PER_USEC)

static struct tt_Node node;
static struct tt_Publisher pub;
static struct tt_Subscriber sub;

static void ping_callback(struct tt_Subscriber *sub, uint64_t timestamp, uint16_t seq_no, struct PerfData *data)
{
	if (data->op != TICKLE_PERF_PING) {
		return;
	}

	if ((data->id - 1) % 6 + 1 != sub->node->id - 1) {
		return;
	}

	data->op = TICKLE_PERF_PONG;
	int ret = tt_Publisher_publish(&pub, data);
	if (ret != 0) {
		printk("Failed to publish pong: %d\n", ret);
		return;
	}
}

int pong_main(void)
{
	printk("Pong Node Started\n");

	PerfTopic.name = "PerfTopic",
	PerfTopic.data_size = sizeof(struct PerfData),
	PerfTopic.data_encode_size = (tt_DATA_ENCODE_SIZE)PerfData_encode_size,
	PerfTopic.data_encode = (tt_DATA_ENCODE)PerfData_encode,
	PerfTopic.data_decode = (tt_DATA_DECODE)PerfData_decode,
	PerfTopic.data_free = (tt_DATA_FREE)PerfData_free,

	_tt_CONFIG.broadcast = "192.168.10.255";

	int32_t ret = tt_Node_create(&node);
	if (ret != 0) {
		printk("Failed to create node: %d\n", ret);
		return ret;
	}

	printk("Pong Node Created, Node ID: %d\n", node.id);

	ret = tt_Node_create_publisher(&node, &pub, &PerfTopic, "perf_data_topic");
	if (ret != 0) {
		printk("Failed to create publisher: %d\n", ret);
		return ret;
	}
	printk("Publisher Created\n");

	ret = tt_Node_create_subscriber(&node, &sub, &PerfTopic, "perf_data_topic",
					(tt_SUBSCRIBER_CALLBACK)ping_callback);
	if (ret != 0) {
		printk("Failed to create subscriber: %d\n", ret);
		return ret;
	}
	printk("Subscriber Created\n");

	tt_Node_poll(&node);

	tt_Node_destroy(&node);

	return 0;
}
