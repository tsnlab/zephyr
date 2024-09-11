/*
***************************************************************************************************
*
*   FileName : clock.h
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

#ifndef MCU_BSP_CLOCK_HEADER
#define MCU_BSP_CLOCK_HEADER

#include <sal_internal.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

#define CKC_MHZ(x)                      (uint32)((x) * 1000000)

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

void CLOCK_Init
(
    void
);

sint32 CLOCK_SetPllRate
(
    sint32                              iId,
    uint32                              uiRate
);

uint32 CLOCK_GetPllRate
(
    sint32                              iId
);

sint32 CLOCK_SetPllDiv
(
    sint32                              iId,
    uint32                              uiPllDiv
);

sint32 CLOCK_SetClkCtrlRate
(
    sint32                              iId,
    uint32                              uiRate
);

uint32 CLOCK_GetClkCtrlRate
(
    sint32                              iId
);

sint32 CLOCK_IsPeriEnabled
(
    sint32                              iId
);

sint32 CLOCK_EnablePeri
(
    sint32                              iId
);

sint32 CLOCK_DisablePeri
(
    sint32                              iId
);

uint32 CLOCK_GetPeriRate
(
    sint32                              iId
);

sint32 CLOCK_SetPeriRate
(
    sint32                              iId,
    uint32                              uiRate
);

sint32 CLOCK_IsIobusPwdn
(
    sint32                              iId
);

sint32 CLOCK_EnableIobus
(
    sint32                              iId,
    boolean                             bEn
);

sint32 CLOCK_SetIobusPwdn
(
    sint32                              iId,
    boolean                             bEn
);

sint32 CLOCK_SetSwReset
(
    sint32                              iId,
    boolean                             bReset
);

#endif  // MCU_BSP_CLOCK_HEADER

