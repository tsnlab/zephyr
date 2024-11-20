/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <stdio.h>
#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>
#include <errno.h>
#include <ctype.h>

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/console/console.h>
#include <zephyr/drivers/console/uart_console.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/pm/device_runtime.h>
#ifdef CONFIG_UART_CONSOLE_MCUMGR
#include <zephyr/mgmt/mcumgr/transport/serial.h>
#endif

#if 0
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/devicetree.h>
#include <zephyr/cache.h>
#endif

//#include <adsp_memory.h>
//#include <mem_window.h>

static const struct device *const uart_console_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

//const struct device *tccvcp_console_dev;

static int arch_printk_char_out(int c)
{
#if 1
    if (pm_device_runtime_get(uart_console_dev) < 0) {
        /* Enabling the UART instance has failed but this
         * function MUST return the byte output.
         */
        return c;
    }

    if ('\n' == c) {
        uart_poll_out(uart_console_dev, '\r');
    }
    uart_poll_out(uart_console_dev, c);

    /* Use async put to avoid useless device suspension/resumption
     * when tranmiting chain of chars.
     * As errors cannot be returned, ignore the return value
     */
    (void)pm_device_runtime_put_async(uart_console_dev, K_MSEC(1));

    return c;

#else
    static int flag_ready_console_dev = 0;
	unsigned char s = (unsigned char)(c & 0xFF);

    if(flag_ready_console_dev == 0) {
        tccvcp_console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

        if (!device_is_ready(tccvcp_console_dev)) {
            return -ENODEV;
        }

        flag_ready_console_dev = 1;
    }

	uart_tccvcp_poll_out(tccvcp_console_dev, s);
    if(s == '\n') {
	    uart_tccvcp_poll_out(tccvcp_console_dev, '\r');
    }
	return 0;
#endif
}

int arch_printf_char_out(int c)
{
    return arch_printk_char_out(c);
}

static void tccvcp_console_hook_install(void)
{
#if defined(CONFIG_STDOUT_CONSOLE)
	__stdout_hook_install(arch_printf_char_out);
#endif
#if defined(CONFIG_PRINTK)
	__printk_hook_install(arch_printk_char_out);
#endif
}


static int tccvcp_console_init(void)
{
    if (!device_is_ready(uart_console_dev)) {
        return -ENODEV;
    }

	tccvcp_console_hook_install();

	return 0;
}

//SYS_INIT(tccvcp_console_init, PRE_KERNEL_2, CONFIG_CONSOLE_INIT_PRIORITY);
SYS_INIT(tccvcp_console_init, POST_KERNEL, CONFIG_CONSOLE_INIT_PRIORITY);
