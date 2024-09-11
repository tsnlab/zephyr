/*
***************************************************************************************************
*
*   FileName : bsp.c
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

#include "bsp.h"

#include <reg_phys.h>
#include <debug.h>

#include "clock.h"
#include "gic.h"
#include "gpio.h"

#if 1 //( MCU_BSP_SUPPORT_DRIVER_UART == 1 )
    #include "uart.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_UART == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_TIMER == 1 )
    #include "timer.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_TIMER == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_WATCHDOG == 1 )
    #include <wdt.h>
#endif  // ( MCU_BSP_SUPPORT_DRIVER_WATCHDOG == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_I2C == 1 )
    #include "i2c.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_I2C == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_GPSB == 1 )
    #include "gpsb.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_GPSB == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_ADC == 1 )
    #include <adc.h>
#endif  // ( MCU_BSP_SUPPORT_DRIVER_ADC == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_MBOX == 1 )
    #include "mbox.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_MBOX == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_PMU == 1 )
    #include <pmu.h>
#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMU == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )
    #include <pmio.h>
#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_DSE == 1 )
    #include "dse.h"
#endif  // ( MCU_BSP_SUPPORT_DRIVER_DSE == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_FMU == 1 )
    #include <fmu.h>
#endif  // ( MCU_BSP_SUPPORT_DRIVER_FMU == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_RTC == 1 )
    #include <rtc.h>
#endif  // MCU_BSP_SUPPORT_DRIVER_RTC == 1 )

/*
***************************************************************************************************
                                         STATIC VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/

void BSP_PreInit
(
    void
)
{
    CLOCK_Init();
    GIC_Init();

#if ( MCU_BSP_SUPPORT_DRIVER_PMU == 1 )
    PMU_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMU == 1 )

    BSP_EnableSYSPower();
}

void BSP_Init
(
    void
)
{
#if ( MCU_BSP_SUPPORT_DRIVER_UART == 1 )
    (void)UART_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_UART == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_TIMER == 1 )
    (void)TIMER_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_TIMER == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_WATCHDOG == 1 )
    //(void)WDT_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_WATCHDOG == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_DSE == 1 )
    (void)DSE_Init(DES_SEL_ALL);
#endif  // ( MCU_BSP_SUPPORT_DRIVER_DSE == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_I2C == 1 )
    I2C_Init();
#endif // (MCU_BSP_SUPPORT_DRIVER_I2C == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_GPSB == 1 )
    GPSB_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_GPSB == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_ADC == 1 )
    (void) ADC_Init(ADC_MODE_NORMAL,ADC_MODULE_0);
    (void) ADC_Init(ADC_MODE_NORMAL,ADC_MODULE_1);
#endif  // ( MCU_BSP_SUPPORT_DRIVER_ADC == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_FMU == 1 )
    (void)FMU_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_FMU == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_RTC == 1 )
    (void)RTC_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_RTC == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_MBOX == 1 )
    MBOX_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_MBOX == 1 )

#if ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )
    PMIO_Init();
#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )


    mcu_printf("\nInitialize System done\n");
    mcu_printf("Welcome to Telechips MCU BSP\n");
}

/* R0 : ARM Exception ID, R1 : Dummy , R2 : Link Register(Return PC)*/
void BSP_UndefAbortExceptionHook
(
    uint32                              uiExceptId,
    uint32                              uiDummy,
    uint32                              uiLinkReg
)
{
    (void)uiExceptId;
    (void)uiDummy;
    (void)uiLinkReg;

}

/*
Warning !!!  After calliing this function. it will never return from exception except H/W reseting.
This function is designed for very restricted use case such as firmware upgrading.
*/
void BSP_SystemHalt
(
    void
)
{
    (void)SAL_CoreCriticalEnter();  // Interrupt disable(I/F)
    ARM_Reserved();                 // move ARM p.c on the sram.
}

void BSP_EnableSYSPower
(
    void
)
{
    /* Enable SYS_3P3 */
    (void) GPIO_Config(SYS_PWR_EN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void) GPIO_Set(SYS_PWR_EN, 1UL);
}

