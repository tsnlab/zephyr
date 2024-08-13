#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define THREAD_STACK_SIZE	1024
#define THREAD_PRIORITY		1
#define SLEEPTIME_MS		100
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
static k_tid_t sub_thread_tid;

void stress_thread(void);
K_THREAD_STACK_DEFINE(stress_thread_stack_area, THREAD_STACK_SIZE);
static struct k_thread stress_thread_metadata;
static k_tid_t stress_thread_tid;



/***************************************************
 *
 * Main
 *
 ***************************************************/
int main(void)
{
	uint64_t curr_ts;

	k_tid_t sub_thread_tid = k_thread_create(
			&sub_thread_metadata, sub_thread_stack_area,
			K_THREAD_STACK_SIZEOF(sub_thread_stack_area),
			sub_thread, NULL, NULL, NULL,
			THREAD_PRIORITY, 0, K_FOREVER
			);
	k_tid_t stress_thread_tid = k_thread_create(
			&stress_thread_metadata, stress_thread_stack_area,
			K_THREAD_STACK_SIZEOF(stress_thread_stack_area),
			stress_thread, NULL, NULL, NULL,
			THREAD_PRIORITY, 0, K_FOREVER
			);

	k_thread_name_set(&sub_thread_metadata, "sub_thread");
	k_thread_name_set(&stress_thread_metadata, "stress_thread");

	k_thread_start(&sub_thread_metadata);
	k_thread_start(&stress_thread_metadata);

	curr_ts = get_timestamp_ns();
	for(;;) {
		/* 100ms interval */
		if((get_timestamp_ns() - curr_ts) >= SLEEPTIME_NS){
			k_wakeup(sub_thread_tid);
			curr_ts = get_timestamp_ns();
		}

		/*
		 * one tick is 10ms in qemu_x86.
		 * so, k_usleep(1) sleeps for 10ms on qemu_x86.
		 */
		k_usleep(1);
	}

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
	printk("start sub_thread()\n");
	uint64_t ts1, ts2;
	for(int i=0; i<1010; i++){
		ts1 = get_timestamp_ns();
		printk("%llu\n", ts1);
		ts2 = get_timestamp_ns();
		k_sleep(Z_TIMEOUT_NS(SLEEPTIME_NS - (ts2 - ts1)));
	}
}

void stress_thread()
{
	printk("start stress_thread()\n");
	for(int i=0;;) {
		i++;
		k_usleep(1);
	}
}
