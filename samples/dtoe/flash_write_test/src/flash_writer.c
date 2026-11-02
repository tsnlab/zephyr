/*
 * Erases and writes one 4 KB sector of the storage partition every FLASH_WRITE_PERIOD_MS, the next sector each
 * time, and reads it back. Runs above the priority of main (the streaming loop), so it preempts it.
 * Every 5 s: [flash] n=<cycles> erase_max=<us> write_max=<us> err=<count>
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/storage/flash_map.h>

#define FLASH_WRITE_PERIOD_MS 1000
#define FLASH_REPORT_MS       5000
#define SECTOR_SIZE           4096

static uint8_t pattern[SECTOR_SIZE];
static uint8_t readback[SECTOR_SIZE];

static void flash_writer(void *a, void *b, void *c)
{
	const struct flash_area *fa;
	uint32_t sectors, sector = 0, cycles = 0, errors = 0;
	uint32_t erase_max = 0, write_max = 0;
	int64_t next_report;

	if (flash_area_open(PARTITION_ID(storage_partition), &fa) != 0) {
		printk("[flash] cannot open storage partition\n");
		return;
	}
	/* the image (code and initial data) must end below the storage */
	if ((uintptr_t)__rom_region_start + (uintptr_t)_flash_used > CONFIG_FLASH_BASE_ADDRESS + fa->fa_off) {
		printk("[flash] image overlaps the storage partition\n");
		return;
	}
	sectors = fa->fa_size / SECTOR_SIZE;
	printk("[flash] writer: %u sectors at 0x%lx, one every %d ms\n", sectors, (unsigned long)fa->fa_off,
	       FLASH_WRITE_PERIOD_MS);
	/* let DTOE start and the peer match first */
	k_msleep(10000);
	next_report = k_uptime_get() + FLASH_REPORT_MS;

	for (;;) {
		off_t off = (off_t)sector * SECTOR_SIZE;
		uint32_t t0, t1, t2;

		for (uint32_t i = 0; i < SECTOR_SIZE; i++) {
			pattern[i] = (uint8_t)(i + cycles);
		}
		t0 = k_cycle_get_32();
		if (flash_area_erase(fa, off, SECTOR_SIZE) != 0) {
			errors++;
		}
		t1 = k_cycle_get_32();
		if (flash_area_write(fa, off, pattern, SECTOR_SIZE) != 0) {
			errors++;
		}
		t2 = k_cycle_get_32();
		if (flash_area_read(fa, off, readback, SECTOR_SIZE) != 0 ||
		    memcmp(pattern, readback, SECTOR_SIZE) != 0) {
			errors++;
		}
		erase_max = MAX(erase_max, k_cyc_to_us_floor32(t1 - t0));
		write_max = MAX(write_max, k_cyc_to_us_floor32(t2 - t1));
		cycles++;
		sector = (sector + 1) % sectors;

		if (k_uptime_get() >= next_report) {
			printk("[flash] n=%u erase_max=%uus write_max=%uus err=%u\n", cycles, erase_max, write_max,
			       errors);
			erase_max = write_max = 0;
			next_report += FLASH_REPORT_MS;
		}
		k_msleep(FLASH_WRITE_PERIOD_MS);
	}
}

/* cooperative, above main: preempts the busy-waiting streaming loop */
K_THREAD_DEFINE(flash_writer_tid, 2048, flash_writer, NULL, NULL, NULL, -1, 0, 0);
