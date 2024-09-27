
//#include <debug.h>
//#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <stdlib.h>
//#include <sal_internal.h>

//#include <bsp.h>
//#include <mpu.h>
#include "preload.h"

extern unsigned long __PRECODE_START_LOAD;
extern unsigned long __PRECODE_RAM_START__;
extern unsigned long __PRECODE_SIZE__;

#if 1
void PRELOAD_loadOnRam(void)
{
	unsigned long rompos = (unsigned long)&__PRECODE_START_LOAD;
	unsigned long rampos = (unsigned long)&__PRECODE_RAM_START__;
	unsigned long sfmc_code_end = (unsigned long)&__PRECODE_SIZE__;
	unsigned long readedvalue;

	while (rampos < sfmc_code_end) {
		readedvalue = sys_read32(rompos);
		sys_write32(readedvalue, rampos);

		if (rompos < 0x10000000u) // to prevent cert_int30_c_violation
		{
			rompos = rompos + 4u;
		}
		rampos = rampos + 4u;
	}

	return;
}
#endif

void PRELOAD_JOB(void)
{
	/*
	   Change the PLL rate for low current
	*/

	unsigned long uiRegDt;

	// HSM_CLK to XIN
	sys_write32(0x0UL, 0xA0F2401CUL);

	// CPU/BUS/EFL_CLK to XIN
	sys_write32(0x0UL, 0xA0F24020UL);

	// SFMC_CLK to XIN
	uiRegDt = (sys_read32(0xA0F24028UL) & 0x9FFFFFFFUL);
	sys_write32(uiRegDt, 0xA0F24028UL);

	uiRegDt = (sys_read32(0xA0F24028UL) & 0xE0FFF000UL);
	uiRegDt |= (5UL << 24UL);
	sys_write32(uiRegDt, 0xA0F24028UL);

	uiRegDt = (sys_read32(0xA0F24028UL) | (3UL << 29UL));
	sys_write32(uiRegDt, 0xA0F24028UL);

	// Disable SRC_CLK_DIV
	sys_write32(0x0UL, 0xA0F24018UL);

	// Drop  PLL0 : 1200 -> 600  MHz
	uiRegDt = (sys_read32(0xA0F24000UL) & 0xFFFC7FFF);
	uiRegDt |= (0x1UL << 15UL);
	sys_write32(uiRegDt, 0xA0F24000UL);

	// Check to start PLL0
	while ((sys_read32(0xA0F24000UL) & (0x1UL << 31UL)) == 0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}

	// Drop  PLL1 : 1500 -> 750  MHz
	uiRegDt = (sys_read32(0xA0F2400CUL) & 0xFFFC7FFF);
	uiRegDt |= (0x1UL << 15UL);
	sys_write32(uiRegDt, 0xA0F2400CUL);

	// Check to start PLL1
	while ((sys_read32(0xA0F2400CUL) & (0x1UL << 31UL)) == 0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}

	// Enable SRC_CLK_DIV
	sys_write32(0x81818100UL, 0xA0F24018UL);

	// Check to start PLL0/1 XIN Div
	while ((sys_read32(0xA0F24018UL) & ((0x1UL << 30UL) | (0x1UL << 22UL) | (0x1UL << 14UL))) !=
	       0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}

	// Reset HSM rate (200 -> 187.5 MHz)
	uiRegDt = ((2 << 4UL) | (1 << 0UL));
	sys_write32(uiRegDt, 0xA0F2401CUL);

	// Check to start HSM
	while ((sys_read32(0xA0F2401CUL) & (0x1UL << 7UL)) != 0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}

	// Reset CPU/BUS/EFL rate (300   / 187.5 / 150   MHz)
	uiRegDt = ((1 << 28UL) | (1 << 24UL)); // EFL
	uiRegDt |= ((2 << 12UL) | (1 << 8UL)); // BUS
	uiRegDt |= ((1 << 4UL) | (0 << 0UL));  // CPU
	sys_write32(uiRegDt, 0xA0F24020UL);

	// Check to start CPU/BUSS/EFL
	while ((sys_read32(0xA0F24020UL) & ((0x1UL << 31UL) | (0x1UL << 15UL) | (0x1UL << 7UL))) !=
	       0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}

	// Reset SFMC rate (83.3)
	uiRegDt = (sys_read32(0xA0F24028UL) & 0x9FFFFFFFUL);
	sys_write32(uiRegDt, 0xA0F24028UL); // disable

	uiRegDt = (sys_read32(0xA0F24028UL) & 0xE0FFF000UL);
	uiRegDt |= ((1UL << 24UL) | (8UL << 0UL));
	sys_write32(uiRegDt, 0xA0F24028UL); // pll1/div

	uiRegDt = (sys_read32(0xA0F24028UL) | (3UL << 29UL));
	sys_write32(uiRegDt, 0xA0F24028UL); // enable

	// Check to start SFMC
	while ((sys_read32(0xA0F24028UL) & (0x1UL << 31UL)) == 0UL) {
		__asm__("nop"); // BSP_NOP_DELAY();
	}
	return;
}
