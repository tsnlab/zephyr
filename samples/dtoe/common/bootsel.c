/*
 * Way back to the RP2040 boot ROM (BOOTSEL, USB mass storage and picotool) for test firmware on a board nobody
 * can reach: Zephyr has no pico-sdk USB reset interface, so `picotool reboot` cannot do it.
 * - the host opens the console (CDC ACM) at 1200 baud
 * - or BOOTSEL_FALLBACK_MINUTES after boot, whatever the application does
 */
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define BOOTSEL_BAUD             1200
#define BOOTSEL_FALLBACK_MINUTES 20

// RP2040 boot ROM function lookup (datasheet 2.8.3)
typedef void *(*rom_table_lookup_fn)(const uint16_t *table, uint32_t code);
typedef void (*rom_reset_usb_boot_fn)(uint32_t gpio_activity_pin_mask, uint32_t disable_interface_mask);

static void reset_to_bootsel(void)
{
	rom_table_lookup_fn lookup = (rom_table_lookup_fn)(uintptr_t)*(const uint16_t *)0x18;
	const uint16_t *table = (const uint16_t *)(uintptr_t)*(const uint16_t *)0x14;
	rom_reset_usb_boot_fn reset_usb_boot = (rom_reset_usb_boot_fn)lookup(table, 'U' | ('B' << 8));

	printk("rebooting to BOOTSEL\n");
	k_msleep(100);
	(void)irq_lock();
	reset_usb_boot(0, 0);
}

static void bootsel_thread(void *a, void *b, void *c)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int64_t deadline = k_uptime_get() + BOOTSEL_FALLBACK_MINUTES * 60 * 1000;

	for (;;) {
		uint32_t baud = 0;

		if (uart_line_ctrl_get(console, UART_LINE_CTRL_BAUD_RATE, &baud) == 0 && baud == BOOTSEL_BAUD) {
			reset_to_bootsel();
		}
		if (k_uptime_get() > deadline) {
			reset_to_bootsel();
		}
		k_msleep(100);
	}
}

K_THREAD_DEFINE(bootsel, 1024, bootsel_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
