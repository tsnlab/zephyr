#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define THREAD_STACK_SIZE	1024
#define THREAD_PRIORITY		1
#define SLEEPTIME_NS		100000000

/***************************************************
 *
 * Define function and thread
 *
 ***************************************************/
uint64_t get_timestamp_ns(void);

#if 0
// Static thread
void sub_thread(void);
K_THREAD_DEFINE(sub_thread, THREAD_STACK_SIZE, sub_thread, NULL, NULL, NULL, NULL, 0, 0);
extern const k_tid_t sub_thread;
#endif

// Dynamic thread
void sub_thread(void);
K_THREAD_STACK_DEFINE(sub_thread_stack_area, THREAD_STACK_SIZE);
static struct k_thread sub_thread_metadata;





/***************************************************
 *
 * Main
 *
 ***************************************************/
int main(void)
{
	k_thread_create(&sub_thread_metadata, sub_thread_stack_area,
			K_THREAD_STACK_SIZEOF(sub_thread_stack_area),
			sub_thread, NULL, NULL, NULL,
			THREAD_PRIORITY, 0, K_FOREVER /* K_NO_WAIT : No need k_thread_start() */ );
	k_thread_name_set(&sub_thread_metadata, "sub_thread");
	k_thread_start(&sub_thread_metadata);
	return 0;
}





/***************************************************
 *
 * Implemetation function and thread
 *
 ***************************************************/
uint64_t get_timestamp_ns(void)
{
	/*
	int64_t ts_tick = k_uptime_ticks();
	int64_t ts_ms = k_uptime_get();
	printk("ts_tick : %lld\n", ts_tick);
	printk("ts_ms : %lld\n\n", ts_ms);
	*/

	return k_cyc_to_ns_ceil64(k_cycle_get_64());
}

void sub_thread()
{
	uint64_t ts1, ts2;
	for(int i=0; i<1010; i++){
		ts1 = get_timestamp_ns();
		printk("%llu\n", ts1);
		ts2 = get_timestamp_ns();
		k_sleep(Z_TIMEOUT_NS(SLEEPTIME_NS - (ts2 - ts1)));
	}
}
