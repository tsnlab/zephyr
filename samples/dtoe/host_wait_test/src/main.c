/*
 * Other threads run while a DTOE host API call waits for core 1.
 *
 * main calls dtoe_write() back to back for RUN_MS (each call goes to core 1 and waits for its result; without a
 * remote reader the result is RET_PRECONDITION_NOT_MET). A thread of lower priority counts meanwhile. If the host
 * API waited by spinning, main would never give the CPU away and the counter would stay at 0.
 * Then main sleeps for RUN_MS and the counter runs alone, for comparison.
 *
 * Result line: [wait] calls=<host API calls> counter_during_calls=<n> counter_alone=<n> -> PASS|FAIL
 */
#include <stdio.h>

#include <zephyr/kernel.h>

#include "dtoe_host.h"
#include "msg_joint_state.h"

#define RUN_MS 2000

static volatile uint32_t counter;

static void counter_thread(void *a, void *b, void *c)
{
	for (;;) {
		counter++;
	}
}

/* below main (priority 0), so it runs only when main waits */
K_THREAD_DEFINE(counter_tid, 512, counter_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
	dtoe_handle_t handle;
	msg_joint_state_t state = {0};

	if (dtoe_init(&handle) != RET_OK || dtoe_writer_create(&handle, "joint_state_topic", &msg_joint_state,
							       RELIABILITY_BEST_EFFORT) != RET_OK) {
		printf("[wait] DTOE initialization failed\n");
		return 1;
	}
	/* let the console come up */
	k_msleep(3000);

	for (;;) {
		uint32_t calls = 0, during, alone;
		int64_t end = k_uptime_get() + RUN_MS;

		counter = 0;
		while (k_uptime_get() < end) {
			(void)dtoe_write(&handle, &state, sizeof(state));
			calls++;
		}
		during = counter;
		counter = 0;
		k_msleep(RUN_MS);
		alone = counter;
		printf("[wait] calls=%u counter_during_calls=%u counter_alone=%u -> %s\n", calls, during, alone,
		       during > 0 ? "PASS" : "FAIL");
	}
}
