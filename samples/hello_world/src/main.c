/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
