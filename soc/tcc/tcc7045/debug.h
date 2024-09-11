/*
***************************************************************************************************
*
*   FileName : debug.h
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

#ifndef MCU_BSP_DEBUG_HEADER
#define MCU_BSP_DEBUG_HEADER

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
//#define   DEBUG_ENABLE                    1

#define DBG_MAX_TAGS                    (27UL)
#define DBG_ZERO                        (0UL)
#define DBG_ENABLE                      (0x01U)
#define DBG_DISABLE                     (0x00U)
#define DBG_ENABLE_ALL                  (0xFFFFFFFFUL)
#define DBG_DISABLE_ALL                 (0x00000000UL)
#define DBG_CNT_TO_IDX(c)               ((c) - 0x01U)

#define DBG_EQUALS(a, b)                ((a) == (b))
#define DBG_VALID_ARGCOUNT(d)           (((d) > 0x00UL) && ((d) < 0x04UL))
#define DBG_VALID_TAGIDX(d)             ((d) < DBG_MAX_TAGS)
#define DBG_TAG_BIT(d)                  (((uint32)1UL) << (d))
#define DBG_CHECK_BIT                   (0x00000001U)
#define DBG_LEVEL_POSITION              (0xC0000000U)
#define DBG_LEVEL_DEBUG                 (0x40000000U)
#define DBG_LEVEL_ERROR                 (0x00000000U)
#define DBG_LOG_ENABLEALL               (0xFFFFFFFFU)
#define DBG_LOG_DISABLEALL              (0x00000000U)
#define DBG_CHECK_LEVEL                 (30U)


#define DBG_HELP_LEN                    (4)
#define DBG_ON_LEN                      (2)
#define DBG_OFF_LEN                     (3)
#define DBG_LEVEL_LEN                   (5)
#define DBG_INFO_LEN                    (4)

#define DBG_ERR_NONE                    (0)

#define DBG_TAG_SAL                     (0U)
#define DBG_TAG_MPU                     (1U)
#define DBG_TAG_GIC                     (2U)
#define DBG_TAG_GMAC                    (3U)
#define DBG_TAG_MIDF                    (4U)
#define DBG_TAG_CAN                     (5U)
#define DBG_TAG_GPSB                    (6U)
#define DBG_TAG_UART                    (7U)
#define DBG_TAG_I2C                     (8U)
#define DBG_TAG_PDM                     (9U)
#define DBG_TAG_ICTC                    (10U)
#define DBG_TAG_GDMA                    (11U)
#define DBG_TAG_ADC                     (12U)
#define DBG_TAG_MBOX                    (13U)
#define DBG_TAG_TIMER                   (14U)
#define DBG_TAG_WDT                     (15U)
#define DBG_TAG_CKC                     (16U)
#define DBG_TAG_GPIO                    (17U)
#define DBG_TAG_PMIO                    (18U)
#define DBG_TAG_DSE                     (19U)
#define DBG_TAG_SDM                     (20U)
#define DBG_TAG_FMU                     (21U)
#define DBG_TAG_FWUD                    (22U)
#define DBG_TAG_LIN                     (23U)
#define DBG_TAG_I2S                     (24U)
#define DBG_TAG_EFLASH                  (25U)
#define DBG_TAG_SSM                     (26U)
#define DBG_TAG_SFMC                    (27U)
#define DBG_TAG_SWL                     (28U)




typedef struct DbgTag {
    uint8                               dlTag[6];
} DbgTag_t;

#define mcu_printf                          (DBG_SerialPrintf)

#if (DEBUG_ENABLE)
#   define LOGE(idx, fmt, ...)                                                                            \
    {                                                                                                     \
        if (((gDebugOption >> (idx)) & DBG_CHECK_BIT) != 0U)                                              \
           { mcu_printf("(E)[%-5s][%s:%d] "fmt"\n", dbgTags[(idx)].dlTag, __func__, __LINE__, ##__VA_ARGS__); } \
        else {;}                                                                                          \
    }

#   define LOGD(idx, fmt, ...)                                                                            \
    {                                                                                                     \
        if (((gDebugOption >> DBG_CHECK_LEVEL) & (gDebugOption >> (idx)) & DBG_CHECK_BIT) != 0U)          \
          {  mcu_printf("(D)[%-5s][%s:%d] "fmt"\n", dbgTags[(idx)].dlTag, __func__, __LINE__, ##__VA_ARGS__); } \
        else {;}                                                                                          \
    }
#else
#   define LOGE(idx, fmt, ...)
#   define LOGD(idx, fmt, ...)
#endif

/*
***************************************************************************************************
*                                             GLOBAL VARIABLES
***************************************************************************************************
*/
extern uint32                           gDebugOption;
extern DbgTag_t                         dbgTags[DBG_MAX_TAGS];

/*
***************************************************************************************************
*                                             FUNCITONS PROTOTYPES
***************************************************************************************************
*/
extern void DBG_SerialPrintf
(
    const int8 *                        format,
    ...
);

#endif  // MCU_BSP_DEBUG_HEADER

