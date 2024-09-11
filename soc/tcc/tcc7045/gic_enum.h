/*
***************************************************************************************************
*
*   FileName : gic_enum.h
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

#ifndef MCU_BSP_GIC_ENUM_HEADER
#define MCU_BSP_GIC_ENUM_HEADER

#if 1 // ( MCU_BSP_SUPPORT_DRIVER_GIC == 1 )

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

#define GIC_INT_TYPE_LEVEL_HIGH         (0x1U)
#define GIC_INT_TYPE_LEVEL_LOW          (0x2U)
#define GIC_INT_TYPE_EDGE_RISING        (0x4U)
#define GIC_INT_TYPE_EDGE_FALLING       (0x8U)
#define GIC_INT_TYPE_EDGE_BOTH          (GIC_INT_TYPE_EDGE_RISING | GIC_INT_TYPE_EDGE_FALLING)

#define GIC_PRIORITY_NO_MEAN            (16UL)

#define GIC_PPI_START                   (16UL)
#define GIC_SPI_START                   (32UL)

#define GIC_EINT_START_INT              (GIC_EXT0)
#define GIC_EINT_END_INT                (GIC_EXT9)
#define GIC_EINT_NUM                    ((uint32)GIC_EXTn0 - (uint32)GIC_EXT0)

#define GIC_INT_SRC_CNT                 (GIC_SPU+1UL)



enum
{
    GIC_CAN0_0                          = (GIC_SPI_START + 0UL),
    GIC_CAN0_1                          = (GIC_SPI_START + 1UL),
    GIC_CAN1_0                          = (GIC_SPI_START + 2UL),
    GIC_CAN1_1                          = (GIC_SPI_START + 3UL),
    GIC_CAN2_0                          = (GIC_SPI_START + 4UL),
    GIC_CAN2_1                          = (GIC_SPI_START + 5UL),
    GIC_GPSB                            = (GIC_SPI_START + 6UL),
    GIC_GPSB1                           = (GIC_SPI_START + 7UL),
    GIC_GPSB2                           = (GIC_SPI_START + 8UL),
    GIC_GPSB3                           = (GIC_SPI_START + 9UL),
    GIC_GPSB4                           = (GIC_SPI_START + 10UL),
    GIC_GPSB0_DMA                       = (GIC_SPI_START + 11UL),
    GIC_GPSB1_DMA                       = (GIC_SPI_START + 12UL),
    GIC_GPSB2_DMA                       = (GIC_SPI_START + 13UL),
    GIC_GPSB3_DMA                       = (GIC_SPI_START + 14UL),
    GIC_GPSB4_DMA                       = (GIC_SPI_START + 15UL),
    GIC_UART0                           = (GIC_SPI_START + 16UL),
    GIC_UART1                           = (GIC_SPI_START + 17UL),
    GIC_UART2                           = (GIC_SPI_START + 18UL),
    GIC_UART3                           = (GIC_SPI_START + 19UL),
    GIC_UART4                           = (GIC_SPI_START + 20UL),
    GIC_UART5                           = (GIC_SPI_START + 21UL),
    GIC_DMA0                            = (GIC_SPI_START + 22UL),
    GIC_DMA1                            = (GIC_SPI_START + 23UL),
    GIC_DMA2                            = (GIC_SPI_START + 24UL),
    GIC_DMA3                            = (GIC_SPI_START + 25UL),
    GIC_DMA4                            = (GIC_SPI_START + 26UL),
    GIC_DMA5                            = (GIC_SPI_START + 27UL),
    GIC_DMA6                            = (GIC_SPI_START + 28UL),
    GIC_DMA7                            = (GIC_SPI_START + 29UL),
    GIC_I2C                             = (GIC_SPI_START + 30UL),
    GIC_I2C1                            = (GIC_SPI_START + 31UL),
    GIC_I2C2                            = (GIC_SPI_START + 32UL),
    GIC_I2C3                            = (GIC_SPI_START + 33UL),
    GIC_I2C4                            = (GIC_SPI_START + 34UL),
    GIC_I2C5                            = (GIC_SPI_START + 35UL),
    GIC_VS_I2C0                         = (GIC_SPI_START + 36UL),
    GIC_VS_I2C1                         = (GIC_SPI_START + 37UL),
    GIC_VS_I2C2                         = (GIC_SPI_START + 38UL),
    GIC_VS_I2C3                         = (GIC_SPI_START + 39UL),
    GIC_VS_I2C4                         = (GIC_SPI_START + 40UL),
    GIC_VS_I2C5                         = (GIC_SPI_START + 41UL),
    GIC_ICTC0                           = (GIC_SPI_START + 42UL),
    GIC_ICTC1                           = (GIC_SPI_START + 43UL),
    GIC_ICTC2                           = (GIC_SPI_START + 44UL),
    GIC_ICTC3                           = (GIC_SPI_START + 45UL),
    GIC_ICTC4                           = (GIC_SPI_START + 46UL),
    GIC_ICTC5                           = (GIC_SPI_START + 47UL),
    GIC_RED_ICTC0                       = (GIC_SPI_START + 48UL),
    GIC_RED_ICTC1                       = (GIC_SPI_START + 49UL),
    GIC_RED_ICTC2                       = (GIC_SPI_START + 50UL),
    GIC_RED_ICTC3                       = (GIC_SPI_START + 51UL),
    GIC_RED_ICTC4                       = (GIC_SPI_START + 52UL),
    GIC_RED_ICTC5                       = (GIC_SPI_START + 53UL),
    GIC_ADC0                            = (GIC_SPI_START + 54UL),
    GIC_ADC1                            = (GIC_SPI_START + 55UL),
    GIC_TIMER_0                         = (GIC_SPI_START + 56UL),
    GIC_TIMER_1                         = (GIC_SPI_START + 57UL),
    GIC_TIMER_2                         = (GIC_SPI_START + 58UL),
    GIC_TIMER_3                         = (GIC_SPI_START + 59UL),
    GIC_TIMER_4                         = (GIC_SPI_START + 60UL),
    GIC_TIMER_5                         = (GIC_SPI_START + 61UL),
    GIC_TIMER_6                         = (GIC_SPI_START + 62UL),
    GIC_TIMER_7                         = (GIC_SPI_START + 63UL),
    GIC_TIMER_8                         = (GIC_SPI_START + 64UL),
    GIC_TIMER_9                         = (GIC_SPI_START + 65UL),
    GIC_WATCHDOG                        = (GIC_SPI_START + 66UL),
    GIC_PMU_WATCHDOG                    = (GIC_SPI_START + 67UL),
    GIC_DEFAULT_SLV_ERR                 = (GIC_SPI_START + 68UL),
    GIC_SFMC                            = (GIC_SPI_START + 69UL),
    GIC_CR5_PMU                         = (GIC_SPI_START + 70UL),
    GIC_EXT0                            = (GIC_SPI_START + 71UL),
    GIC_EXT1                            = (GIC_SPI_START + 72UL),
    GIC_EXT2                            = (GIC_SPI_START + 73UL),
    GIC_EXT3                            = (GIC_SPI_START + 74UL),
    GIC_EXT4                            = (GIC_SPI_START + 75UL),
    GIC_EXT5                            = (GIC_SPI_START + 76UL),
    GIC_EXT6                            = (GIC_SPI_START + 77UL),
    GIC_EXT7                            = (GIC_SPI_START + 78UL),
    GIC_EXT8                            = (GIC_SPI_START + 79UL),
    GIC_EXT9                            = (GIC_SPI_START + 80UL),
    GIC_EXTn0                           = (GIC_SPI_START + 81UL),
    GIC_EXTn1                           = (GIC_SPI_START + 82UL),
    GIC_EXTn2                           = (GIC_SPI_START + 83UL),
    GIC_EXTn3                           = (GIC_SPI_START + 84UL),
    GIC_EXTn4                           = (GIC_SPI_START + 85UL),
    GIC_EXTn5                           = (GIC_SPI_START + 86UL),
    GIC_EXTn6                           = (GIC_SPI_START + 87UL),
    GIC_EXTn7                           = (GIC_SPI_START + 88UL),
    GIC_EXTn8                           = (GIC_SPI_START + 89UL),
    GIC_EXTn9                           = (GIC_SPI_START + 90UL),
    GIC_FMU_IRQ                         = (GIC_SPI_START + 91UL),
    GIC_FMU_FIQ                         = (GIC_SPI_START + 92UL),
    GIC_I2S_DMA                         = (GIC_SPI_START + 93UL),
    GIC_GMAC                            = (GIC_SPI_START + 94UL),
    GIC_GMAC_TX0                        = (GIC_SPI_START + 95UL),
    GIC_GMAC_TX1                        = (GIC_SPI_START + 96UL),
    GIC_GMAC_RX0                        = (GIC_SPI_START + 97UL),
    GIC_GMAC_RX1                        = (GIC_SPI_START + 98UL),
    GIC_SRAM_ECC                        = (GIC_SPI_START + 99UL),
    GIC_DMAC_                           = (GIC_SPI_START + 100UL),
    GIC_DMAC_INT_CLEAR                  = (GIC_SPI_START + 101UL),
    GIC_DMAC_INTERR                     = (GIC_SPI_START + 102UL),
    GIC_HSM_WDT_OUTPUT                  = (GIC_SPI_START + 103UL),
    GIC_MBOX_SLV_TX                     = (GIC_SPI_START + 104UL),
    GIC_MBOX_MST_TX                     = (GIC_SPI_START + 105UL),
    GIC_AES                             = (GIC_SPI_START + 106UL),
    GIC_TRNG                            = (GIC_SPI_START + 107UL),
    GIC_TRNG_FAD                        = (GIC_SPI_START + 108UL),
    GIC_PKE                             = (GIC_SPI_START + 109UL),
    GIC_PKE_ECC_S                       = (GIC_SPI_START + 110UL),
    GIC_PKE_ECC_D                       = (GIC_SPI_START + 111UL),
    GIC_HSM_DEFAULT_SLV_ERR             = (GIC_SPI_START + 112UL),
    GIC_HSM_CPU_SYS_RESET_REQ           = (GIC_SPI_START + 113UL),
    GIC_PFLASH                          = (GIC_SPI_START + 114UL),
    GIC_DFLASH                          = (GIC_SPI_START + 115UL),
    GIC_RTC_ALARM                       = (GIC_SPI_START + 116UL),
    GIC_RTC_WAKEUP                      = (GIC_SPI_START + 117UL),
    GIC_GMAC_TX2                        = (GIC_SPI_START + 118UL),
    GIC_GMAC_RX2                        = (GIC_SPI_START + 119UL),
    GIC_SFMC1                           = (GIC_SPI_START + 120UL),
    GIC_SPU                             = (GIC_SPI_START + 121UL),
};

#endif  // ( MCU_BSP_SUPPORT_DRIVER_GIC == 1 )

#endif  // MCU_BSP_GIC_ENUM_HEADER

