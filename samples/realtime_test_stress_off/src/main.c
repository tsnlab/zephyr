#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define THREAD_STACK_SIZE	1024
#define THREAD_PRIORITY		1
#define SLEEPTIME_MS		100

/***************************************************
 *
 * Define function and thread
 *
 ***************************************************/
uint64_t get_timestamp_ms(void);

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
uint64_t get_timestamp_ms(void)
{
	/*
	int64_t ts_tick = k_uptime_ticks();
	int64_t ts_ms = k_uptime_get();
	printk("ts_tick : %lld\n", ts_tick);
	printk("ts_ms : %lld\n\n", ts_ms);
	*/

	return k_uptime_get();
}

void sub_thread()
{
	for(int i=0; i<1000; i++){
		printk("%llu\n", get_timestamp_ms()%1000);
		//k_msleep(SLEEPTIME_MS);
		k_busy_wait(SLEEPTIME_MS * 1000);
	}
}
