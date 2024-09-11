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
#include "uart.h"
#include "timer.h"
#include <pmio.h>

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

    BSP_EnableSYSPower();
}

void BSP_Init
(
    void
)
{
    (void)UART_Init();

    (void)TIMER_Init();

    PMIO_Init();

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

