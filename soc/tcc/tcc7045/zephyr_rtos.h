/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_RTOS_H
#define ZEPHYR_RTOS_H

void cpu_dsb(void);

uint32_t cpu_sr_save(void);
void cpu_sr_restore(uint32_t);

#endif /* ZEPHYR_RTOS_H */
