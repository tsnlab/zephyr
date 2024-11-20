# Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
#
# SPDX-License-Identifier: Apache-2.0

.global  CPU_DSB
.global  CPU_SR_Save
.global  CPU_SR_Restore

CPU_DSB:
        DSB
        ISB
        BX      LR

CPU_SR_Save:
        MRS     R0, CPSR
        CPSID   IF                                              @ Set IRQ & FIQ bits in CPSR to DISABLE all interrupts
        DSB
        BX      LR                                              @ DISABLED, return the original CPSR contents in R0

CPU_SR_Restore:
        DSB
        MSR     CPSR_c, R0
        BX      LR

.end
