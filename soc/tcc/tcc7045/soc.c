/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <cmsis_core.h>

void z_arm_platform_init(void)
{
	/*
	 * Use normal exception vectors address range (0x0-0x1C).
	 */
	unsigned int sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_V_Msk;
	__set_SCTLR(sctlr);

#if 1
    __asm__ volatile(".word 0x4855434D");        /* 0x04 : MCUH */
    __asm__ volatile(".word 0x00000000");    /* 0x08 : micom f/w load base */

    // Don't delete below 3 words.
    // This is to apply the 2oo3 voting method.

    __asm__ volatile(".word (0x56010000)");
    __asm__ volatile(".word (0x56010000)");
    __asm__ volatile(".word (0x56010000)");

    __asm__ volatile(".word 0x00000000 ");       /* 0x10 : reserved */
    __asm__ volatile(".word 0x00000000 ");       /* 0x14 : reserved */
    __asm__ volatile(".word 0x00000000 ");       /* 0x18 : reserved */
    __asm__ volatile(".word 0x00000000 ");       /* 0x1c : reserved */
#endif

}
