/*
***************************************************************************************************
*
*   FileName : reg_phys.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*
*   TCC Version 1.0
*
*   This source code contains confidential information of Telechips.
*
*   Any unauthorized use without a written permission of Telechips including not limited to
*   re-distribution in source or binary form is strictly prohibited.
*
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty of
*   merchantability, fitness for a particular purpose or non-infringement of any patent, copyright
*   or other third party intellectual property right. No warranty is made, express or implied,
*   regarding the information's accuracy,completeness, or performance.
*
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between
*   Telechips and Company.
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty
*   (of merchantability, fitness for a particular purpose or non-infringement of any patent,
*   copyright or other third party intellectual property right. No warranty is made, express or
*   implied, regarding the information's accuracy, completeness, or performance.
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement
*   between Telechips and Company.
*
***************************************************************************************************
*/

#ifndef MCU_BSP_REG_PHYS_HEADER
#define MCU_BSP_REG_PHYS_HEADER

/* Part 11. MICOM */
#define MCU_BSP_CAN_FD_BASE             (0xA0000000UL)
#define MCU_BSP_ADC_0_BASE              (0xA0080000UL)
#define MCU_BSP_ADC_1_BASE              (0xA0090000UL)
#define MCU_BSP_GPSB_BASE               (0xA0100000UL)
#define MCU_BSP_UART_BASE               (0xA0200000UL)
#define MCU_BSP_UART_CONF_BASE          (0xA02F0000UL)
#define MCU_BSP_I2C_BASE                (0xA0400000UL)
#define MCU_BSP_ICTC_BASE               (0xA0600000UL)
#define MCU_BSP_PWM_BASE                (0xA0700000UL)
#define MCU_BSP_PWM_CFG_BASE            (0xA07F0000UL)
#define MCU_BSP_GDMA_BASE               (0xA0800000UL)
#define MCU_BSP_I2S_BASE                (0xA0880000UL)
#define MCU_BSP_GMAC_BASE               (0xA0900000UL)
#define MCU_BSP_HSM_BASE                (0xA0980000UL)
#define MCU_BSP_CRYPTO_BASE             (0xA09B0000UL)
#define MCU_BSP_MBOX_BASE               (0xA09F0000UL)

#define MCU_BSP_SFMC_BASE               (0xA0F00000UL)
#define MCU_BSP_GIC_BASE                (0xA0F10000UL)
#define MCU_BSP_SUBSYS_BASE             (0xA0F20000UL)
#define MCU_BSP_R5_CONF_BASE            (0xA0F21000UL)
#define MCU_BSP_GPIO_BASE               (0xA0F22000UL)
#define MCU_BSP_WDT_BASE                (0xA0F23000UL)
#define MCU_BSP_CKC_BASE                (0xA0F24000UL)
#define MCU_BSP_CMU_BASE                (0xA0F25000UL)
#define MCU_BSP_SYSSM_BASE              (0xA0F26000UL)
#define MCU_BSP_DSE_BASE                (0xA0F27000UL)
#define MCU_BSP_PMU_BASE                (0xA0F28000UL)
#define MCU_BSP_FMU_BASE                (0xA0F28400UL)
#define MCU_BSP_PMIO_BASE               (0xA0F28800UL)
#define MCU_BSP_RTC_BASE                (0xA0F28C00UL)
#define MCU_BSP_TIMER_BASE              (0xA0F2A000UL)

/*  for reference address
#define MBOX_CMD_FIFO_TXD_ADDR          (MCU_BSP_MBOX_BASE)
#define MBOX_CMD_FIFO_RXD_ADDR          (MCU_BSP_MBOX_BASE + 0x20UL)
#define MBOX_CTRL_ADDR                  (MCU_BSP_MBOX_BASE + 0x40UL)
#define MBOX_CMD_FIFO_STS_ADDR          (MCU_BSP_MBOX_BASE + 0x44UL)
*/
#endif  // MCU_BSP_REG_PHYS_HEADER

