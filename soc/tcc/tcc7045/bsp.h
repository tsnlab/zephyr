/*
***************************************************************************************************
*
*   FileName : bsp.h
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

#ifndef MCU_BSP_HEADER
#define MCU_BSP_HEADER

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <reg_phys.h>
#include <sal_internal.h>

#ifndef BSP_NOP_DELAY
#define BSP_NOP_DELAY()                 {__asm__ ("nop");}
#endif


#define SYS_PWR_EN                      (GPIO_GPC(2UL))

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          ARM_Reserved
*
* This function is to move ARM p.c on the sram
*
* Notes
*   QAC : reference .asm function in C module
*
***************************************************************************************************
*/

void ARM_Reserved
(
    void
);


/*
***************************************************************************************************
*                                       ARM_DSB
*
* ARM DSB.
*
* @param    none
* @return   void
*
* Notes
*
***************************************************************************************************
*/

void ARM_DSB
(
    void
);

/*
***************************************************************************************************
*                                          BSP_EnableSYSPower
*
* This function is to enable system power
*
* @param    none.
* @return   void
*
* Notes
*
***************************************************************************************************
*/

void BSP_EnableSYSPower
(
    void
);

#endif  // MCU_BSP_HEADER

