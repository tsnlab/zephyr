#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <tickle/tickle.h>
#include "type/Perf.h"

#define PING_INTERVAL_NS (1000ULL * NSEC_PER_MSEC)

static struct tt_Node node;
static struct tt_Publisher pub;
static struct tt_Subscriber sub;

uint32_t last_id = 0;
uint64_t last_timestamp = 0;

static void send_ping(struct tt_Node *node, uint64_t time, void *param)
{
	struct PerfData data = {
		.id = last_id + 1,
		.timestamp = time,
	};
	printk("Sending ping: id=%d, timestamp=%lld\n", data.id, data.timestamp);
	int ret = tt_Publisher_publish(&pub, &data);
	if (ret != 0) {
		printk("Failed to publish ping: %d\n", ret);
		return;
	}
	last_id = data.id;
	last_timestamp = data.timestamp;
	tt_Node_schedule(node, time + PING_INTERVAL_NS, send_ping, NULL);
}

static void pong_callback(struct tt_Subscriber *sub, uint64_t timestamp, uint16_t seq_no, struct PerfData *data)
{
	// printk("Pong received: id=%d, timestamp=%lld\n", data->id, data->timestamp);
	if (data->id != last_id) {
		printk("ERROR: Pong received with incorrect id: expected=%d, actual=%d\n", last_id, data->id);
		return;
	}

	printk("RTT: %lld ns\n", timestamp - data->timestamp);
}

int ping_main(void)
{
	printk("Ping Node Started\n");

	PerfTopic.name = "PerfTopic";
	PerfTopic.data_size = sizeof(struct PerfData);
	PerfTopic.data_encode_size = (tt_DATA_ENCODE_SIZE)PerfData_encode_size;
	PerfTopic.data_encode = (tt_DATA_ENCODE)PerfData_encode;
	PerfTopic.data_decode = (tt_DATA_DECODE)PerfData_decode;
	PerfTopic.data_free = (tt_DATA_FREE)PerfData_free;

	_tt_CONFIG.broadcast = "192.168.10.255";

	int32_t ret = tt_Node_create(&node);
	if (ret != 0) {
		printk("Failed to create node: %d\n", ret);
		return ret;
	}

	printk("Ping Node Created, Node ID: %d\n", node.id);

	ret = tt_Node_create_publisher(&node, &pub, &PerfTopic, "perf_data_topic");
	if (ret != 0) {
		printk("Failed to create publisher: %d\n", ret);
		return ret;
	}
	printk("Publisher Created\n");

	ret = tt_Node_create_subscriber(&node, &sub, &PerfTopic, "perf_data_topic",
					(tt_SUBSCRIBER_CALLBACK)pong_callback);
	if (ret != 0) {
		printk("Failed to create subscriber: %d\n", ret);
		return ret;
	}
	printk("Subscriber Created\n");

	tt_Node_schedule(&node, tt_get_ns() + PING_INTERVAL_NS, send_ping, NULL);

	tt_Node_poll(&node);

	tt_Node_destroy(&node);

	return 0;
}
