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

#ifndef TCC_SOC_CLOCK_HEADER
#define TCC_SOC_CLOCK_HEADER

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

#define CKC_MHZ(x) (uint32_t)((x)*1000000)

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

void CLOCK_Init(void);

signed long CLOCK_SetPllRate(signed long iId, uint32_t uiRate);

uint32_t CLOCK_GetPllRate(signed long iId);

signed long CLOCK_SetPllDiv(signed long iId, uint32_t uiPllDiv);

signed long CLOCK_SetClkCtrlRate(signed long iId, uint32_t uiRate);

uint32_t CLOCK_GetClkCtrlRate(signed long iId);

signed long CLOCK_IsPeriEnabled(signed long iId);

signed long CLOCK_EnablePeri(signed long iId);

signed long CLOCK_DisablePeri(signed long iId);

uint32_t CLOCK_GetPeriRate(signed long iId);

signed long CLOCK_SetPeriRate(signed long iId, uint32_t uiRate);

signed long CLOCK_IsIobusPwdn(signed long iId);

signed long CLOCK_EnableIobus(signed long iId, unsigned char bEn);

signed long CLOCK_SetIobusPwdn(signed long iId, unsigned char bEn);

signed long CLOCK_SetSwReset(signed long iId, unsigned char bReset);

#endif // TCC_SOC_CLOCK_HEADER
