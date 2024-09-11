/*
***************************************************************************************************
*
*   FileName : timer.h
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

#ifndef MCU_BSP_TIMER_HEADER
#define MCU_BSP_TIMER_HEADER

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <sal_internal.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

typedef enum TIMERStartMode
{
    TIMER_START_MAINCNT                 = 0x0UL,
    TIMER_START_ZERO                    = 0x1UL,

} TIMERStartMode_t;

typedef enum TIMERChannel
{
    TIMER_CH_0                          = 0UL,
    TIMER_CH_1                          = 1UL,
    TIMER_CH_2                          = 2UL,
    TIMER_CH_3                          = 3UL,
    TIMER_CH_4                          = 4UL,
    TIMER_CH_5                          = 5UL,
    TIMER_CH_6                          = 6UL,
    TIMER_CH_7                          = 7UL,
    TIMER_CH_8                          = 8UL,
    TIMER_CH_9                          = 9UL,
    TIMER_CH_MAX                        = 10UL,

} TIMERChannel_t;

typedef enum TIMERCounterMode
{
    TIMER_COUNTER_MAIN                  = 0UL,
    TIMER_COUNTER_COMP0                 = 1UL,
    TIMER_COUNTER_COMP1                 = 2UL,
    TIMER_COUNTER_SMALL_COMP            = 3UL,

} TIMERCounterMode_t;

typedef enum TIMEROperationMode
{
    TIMER_OP_FREERUN                    = 0UL,
    TIMER_OP_ONESHOT

} TIMEROpMode_t;

typedef sint32                          (* TIMERHandler)(TIMERChannel_t uiChannel, void * pArgs);

typedef struct TIMERCfgTable
{
    TIMERChannel_t                      ctChannel;
    TIMERStartMode_t                    ctStartMode;
    TIMEROpMode_t                       ctOpMode;
    TIMERCounterMode_t                  ctCounterMode;
    uint32                              ctMainValueUsec;
    uint32                              ctCmp0ValueUsec;
    uint32                              ctCmp1ValueUsec;
    TIMERHandler                        fnHandler;
    void *                              pArgs;

} TIMERConfig_t;


enum
{
    TIMER_ERR_ALREADY_DONE              = (uint32)SAL_DRVID_TMR,
    TIMER_ERR_SET_CLOCK                 = (uint32)SAL_DRVID_TMR + 1UL,
    TIMER_ERR_NOT_INITIALIZED           = (uint32)SAL_DRVID_TMR + 2UL,
    TIMER_ERR_INVALID_PARAM             = (uint32)SAL_DRVID_TMR + 3UL,
    TIMER_ERR_OUTOF_RANGE_CHANNEL       = (uint32)SAL_DRVID_TMR + 4UL,
    TIMER_ERR_OUTOF_COMPARATOR_0        = (uint32)SAL_DRVID_TMR + 5UL,
    TIMER_ERR_OUTOF_COMPARATOR_1        = (uint32)SAL_DRVID_TMR + 6UL,
    TIMER_ERR_NOT_SUPPORT_MODE          = (uint32)SAL_DRVID_TMR + 7UL,
    TIMER_ERR_INVALID_TIME              = (uint32)SAL_DRVID_TMR + 8UL,
    TIMER_ERR_OUTOF_MAINCNT             = (uint32)SAL_DRVID_TMR + 9UL

};//TIMERErrorCode_t

/*
 * Function Indexes
 */
#define TIMER_API_COMMON_INDEX          (0x0)
#define TIMER_API_ENABLE                (TIMER_API_COMMON_INDEX + 0)
#define TIMER_API_ENABLE_MODE           (TIMER_API_COMMON_INDEX + 1)
#define TIMER_API_ENABLE_CFG            (TIMER_API_COMMON_INDEX + 2)
#define TIMER_API_DISABLE               (TIMER_API_COMMON_INDEX + 3)
#define TIMER_API_INIT                  (TIMER_API_COMMON_INDEX + 4)
#define TIMER_API_CLEAR_IRQ             (TIMER_API_COMMON_INDEX + 5)

/*
***************************************************************************************************
*                                         MODULE INTERFACES
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          TIMER_InterruptClear
*
* This function clears the interrupt signal that is occurred when meeting the conditions.
*
* @param    uiChannel [in] The specific channel for TIMER module
* @return
*
* Notes
*   It is called automatically with dealing with TIMER handler.
*   Therefore, user don't have chance to call this function.
*
***************************************************************************************************
*/
SALRetCode_t TIMER_InterruptClear
(
    TIMERChannel_t                      uiChannel
);

/*
***************************************************************************************************
*                                          TIMER_Enable
*
* This function enables the specific TIMER module with TIMER_OP_FREERUN, TIMER_START_ZERO,
* TIMER_COUNTER_COMP0
*
* @param    uiChannel [in] The specific channel for TIMER module
* @param    uiUSec [in] The micro-second that user want to set for interrupt signal
* @param    fnHandler [in] The external handler which user needs to register
* @param    pArgs [in] The pointer of configuration structure table
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t TIMER_Enable
(
    TIMERChannel_t                      uiChannel,
    uint32                              uiUSec,
    TIMERHandler                        fnHandler,
    void *                              pArgs
);

/*
***************************************************************************************************
*                                          TIMER_EnableWithMode
*
*
* This function enables the specific TIMER module with TIMER_START_ZERO, TIMER_COUNTER_COMP0.
*
* @param    uiChannel [in] The specific channel for TIMER module
* @param    uiUSec [in] The micro-second that user want to set for interrupt signal
* @param    uiOpMode [in] Operation mode, Free running or One-time running
* @param    fnHandler [in] The external handler which user needs to register
* @param    pArgs [in] The pointer of configuration structure table
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t TIMER_EnableWithMode
(
    TIMERChannel_t                      uiChannel,
    uint32                              uiUSec,
    TIMEROpMode_t                       uiOpMode,
    TIMERHandler                        fnHandler,
    void *                              pArgs
);

/*
***************************************************************************************************
*                                          TIMER_EnableWithCfg
*
* This function enables the specific TIMER module with configuration structure
*
* @param    pCfg [in] Configuration structure with TIMER module information
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t TIMER_EnableWithCfg
(
    const TIMERConfig_t *               pCfg
);

/*
***************************************************************************************************
*                                          TIMER_Disable
*
* This function disables the specific TIMER module.
*
* @param    uiChannel [in] The specific channel for TIMER module
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t TIMER_Disable
(
    TIMERChannel_t                      uiChannel
);

/*
***************************************************************************************************
*                                          TIMER_Init
*
* This function initializes clock and registers for all channels
*
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t TIMER_Init
(
    void
);

#endif  // MCU_BSP_TIMER_HEADER

