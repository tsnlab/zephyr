/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

int main(void)
{
	int count = 0;

	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

	while (count < 10000000) {
		printk("%d\n", g_hello_cnt);
		count = 0;
		while (count < 1000) {
			count++;
			printk("\rcount: %4d", count);
		}
		printk("\n");
		g_hello_cnt++;
	}

	return 0;
}
