# Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
#
# SPDX-License-Identifier: Apache-2.0

.global  cpu_dsb
.global  cpu_sr_save
.global  cpu_sr_restore

cpu_dsb:
        DSB
        ISB
        BX      LR

cpu_sr_save:
        MRS     R0, CPSR
        CPSID   IF                                              @ Set IRQ & FIQ bits in CPSR to DISABLE all interrupts
        DSB
        BX      LR                                              @ DISABLED, return the original CPSR contents in R0

cpu_sr_restore:
        DSB
        MSR     CPSR_c, R0
        BX      LR

.end
