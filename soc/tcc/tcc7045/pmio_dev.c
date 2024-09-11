/*
***************************************************************************************************
*
*   FileName : pmio_dev.c
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
***************************************************************************************************/

#if 1 // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )

#include <pmio.h>
typedef unsigned long uint32_t;

#include <pmio_dev.h>
#include <reg_phys.h>
#include "bsp.h"
#include <gic.h>
#include <gpio.h>
#include <debug.h>
/*************************************************************************************************
 *                                             DEFINITIONS
 *************************************************************************************************/



/*PMIO EXTRA FEATURE=====================================================*/

#ifdef PMIO_CONF_DEBUG_ALONE
    #define PMIO_FEATURE_USE_DEBUG_ALONE
#endif

#ifdef PMIO_CONF_PREVENT_LEAK
    #define PMIO_FEATURE_LEAKAGE_REDUCTION_GPK_PORT
#endif

#ifdef PMIO_CONF_DET_BU
    #define PMIO_VA_BU_DET                  (4UL)

    #define PMIO_VA_BU_DET_PORT             (PMIO_GPK(PMIO_VA_BU_DET))
    #define PMIO_VA_BU_DET_GPK              (GPIO_GPK(PMIO_VA_BU_DET))
#else
    #define PMIO_VA_BU_DET_PORT             (PMIO_VA_BITCLEAR)
    #define PMIO_VA_BU_DET_GPK              (0UL)
#endif

#ifdef PMIO_CONF_DET_ACC
    //#define PMIO_FEATURE_ACC_TIMER

    #define PMIO_VA_ACC_DET                 (5UL)

    #define PMIO_VA_ACC_DET_PORT            (PMIO_GPK(PMIO_VA_ACC_DET))
    #define PMIO_VA_ACC_DET_GPK             (GPIO_GPK(PMIO_VA_ACC_DET))
#else
    #define PMIO_VA_ACC_DET_PORT            (PMIO_VA_BITCLEAR)
    #define PMIO_VA_ACC_DET_GPK             (0UL)
#endif

#ifdef PMIO_CONF_DET_CAN
    #define PMIO_VA_CAN_DET                 (1UL)

    #define PMIO_VA_CAN_DET_PORT            (PMIO_GPK(PMIO_VA_CAN_DET))
    #define PMIO_VA_CAN_DET_GPK             (GPIO_GPK(PMIO_VA_CAN_DET))
#else
    #define PMIO_VA_CAN_DET_PORT            (PMIO_VA_BITCLEAR)
    #define PMIO_VA_CAN_DET_GPK             (0UL)
#endif

#ifdef PMIO_CONF_LDO_TRIM_CTL
    #define PMIO_FEATURE_USE_TRIM_CTRL

    //#define PMIO_FEATURE_TRIM_MANUAL_CTRL


    #ifdef PMIO_FEATURE_USE_TRIM_CTRL_MANUAL
    #define PMIO_VA_TRIM_AWO_LDO1P0V    (0UL)
    #define PMIO_VA_TRIM_AWO_LDO1P8V    (0UL)
    #define PMIO_VA_TRIM_AWO_IVDu       (0UL)
    #define PMIO_VA_TRIM_AWO_BGR        (0UL)

    #define PMIO_VA_TRIM_SW_IVDh        (0UL)
    #define PMIO_VA_TRIM_SW_IVDu        (0UL)
    #define PMIO_VA_TRIM_SW_LDO1P8V     (0UL)
    #define PMIO_VA_TRIM_SW_LDO1P0V_A   (0UL)
    #define PMIO_VA_TRIM_SW_LDO1P0V_D   (0UL)
    #define PMIO_VA_TRIM_SW_BGR         (0UL)

    #define PMIO_VA_TRIM_AWO_DATA       ((PMIO_VA_TRIM_AWO_LDO1P0V << 18UL) | \
                                        (PMIO_VA_TRIM_AWO_LDO1P8V << 13UL) | \
                                        (PMIO_VA_TRIM_AWO_IVDu << 8UL) | \
                                        (PMIO_VA_TRIM_AWO_BGR << 3UL))

    #define PMIO_VA_TRIM_SW_DATA        ((PMIO_VA_TRIM_SW_IVDh << 25UL) | \
                                        (PMIO_VA_TRIM_SW_IVDu << 20UL) | \
                                        (PMIO_VA_TRIM_SW_LDO1P8V << 15UL) | \
                                        (PMIO_VA_TRIM_SW_LDO1P0V_A << 10UL) | \
                                        (PMIO_VA_TRIM_SW_LDO1P0V_D << 5UL) | \
                                        (PMIO_VA_TRIM_SW_BGR << 0UL))
    #endif
#endif

#ifdef PMIO_CONF_MISC_DEV_FUNC

#endif

#ifdef PMIO_CONF_RTC_WAKE_UP
    /* Support RTC Wake-up */
    #define PMIO_FEATURE_RTC_PMWKUP

    #define PMIO_VA_RTC_LIMIT_SEC       (5UL)
    #define PMIO_VA_RTC_LIMIT_MIN       (0UL)
    #define PMIO_VA_RTC_LIMIT_HOUR      (0UL)
    #define PMIO_VA_RTC_LIMIT_DATE      (0UL)
    #define PMIO_VA_RTC_LIMIT_DAY       (0UL)
    #define PMIO_VA_RTC_LIMIT_MON       (0UL)
    #define PMIO_VA_RTC_LIMIT_YEAR      (0UL)
#endif

/*=======================================================PMIO EXTRA FEATURE*/



#if defined(DEBUG_ENABLE) || defined(PMIO_FEATURE_USE_DEBUG_ALONE)
    #define PMIO_D(args...)             { \
                                            mcu_printf("[PMIO][%s:%d] ",__func__, __LINE__); \
                                            mcu_printf(args); \
                                        }
#else
    #define PMIO_D(args...)
#endif

#define PMIO_E(args...)                 { \
                                            mcu_printf("[PMIO][%s:%d] Error ! ", __func__, __LINE__); \
                                            mcu_printf(args); \
                                        }

#ifdef PMIO_FEATURE_ACC_TIMER
    #include <timer.h>

    /* When the system wakes up in ACC OFF state, it waits for ACC ON signal using timer. */
    /* If ACC ON does not occur during the time limit, suspend mode is entered. */
    #define PMIO_VA_ACC_OFF_TIME_LIMIT      (10UL)
    #define PMIO_VA_TIMER_CH                (1UL)
#endif

#ifdef PMIO_FEATURE_RTC_PMWKUP
    #include <rtc.h>
#endif

#define PMIO_VA_MAX_EXT_NOTI_SIZE            (10UL)/*Max Gic Ext */

typedef struct
{
    uint32_t uiSrc;
    uint8  ucType;
    uint32_t uiGpk;
    uint32_t uiDepth;
}PMIOGpkIntList_t;

typedef struct
{
    uint32_t uiSrc;
    uint8  uiSrcDirect;
    POWERAppHandler fHandler[4]; /*Max GIC INT TYPE size*/
}PMIONotiInt_t;

typedef struct
{
    uint32_t uiTaskId;
    uint32_t uiTaskStack[2048];
    uint32_t uiEventNum;
    uint32_t uiEventId;

    PMIONotiInt_t tPMIONotiIntList[PMIO_VA_MAX_EXT_NOTI_SIZE];
    POWERAppHandler fSelHandler; /*Max GIC INT TYPE size*/
}PMIONoti_t;
/************************************************************************************************/
/*                                             STATIC FUNCTION                                  */
/************************************************************************************************/

static void PMIO_Delay
(
    uint32_t                              uiCnt
);

static void PMIO_SetPort
(
    PMIOPortSel_t                       uiPortType,
    uint32_t                              uiPort32
);

static void PMIO_InterruptHandler
(
    void *                              pArg
);

static void PMIO_SetInterruptHandler
(
    PMIOGpkIntList_t *sIntList
);

static void PMIO_SetGpkInterrupt
(
    uint32_t uiIntSrc,
    uint8  ucIntType,
    uint32_t uiGpk,
    uint32_t uiDepth
);

static void PMIO_ReducePowerLeakageCurrent
(
    uint32_t uiExceptPort
);

static void PMIO_SetTrimData
(
    uint32_t                              uiSwDt,
    uint32_t                              uiAwoDt
);

static void PMIO_EnTrimData
(
    uint32_t                              uiSwEn,
    uint32_t                              uiAwoEn
);

static void PMIO_SetPower
(
    PMIOMode_t       tMode
);

#ifdef PMIO_FEATURE_ACC_TIMER
    static void PMIO_SetTimer
    (
        uint32_t                          uiUsec
    );

    static sint32 PMIO_TimeoutInterruptHandler
    (
        TIMERChannel_t                  iChannel,
        void *                          pArgs
    );

    static uint32_t   gTimerForDbg        = 0UL;
#endif

static PMIOMode_t PMIO_GetBootMode
(
    void
);

#ifdef PMIO_FEATURE_RTC_PMWKUP
    static void PMIO_SetRtcWakeUp
    (
     uint32_t uiSwEn,
     uint32_t uiAwoEn,
     uint32_t uiYear,
     uint32_t uiMon,
     uint32_t uiDay,
     uint32_t uiDate,
     uint32_t uiHour,
     uint32_t uiMin,
     uint32_t uiSec
    );
#endif

static SALRetCode_t PMIO_NotiGpkIrq
(
    uint32_t                              uiGicSrc,
    uint32_t                              uiGicDirect
);

static void PMIO_NotiTask
(
    void *                              pArg
);

//for debugging power status
static uint32_t   gModeForDbg          = 0UL;
static uint32_t   gReasonForDbg          = 0UL;

PMIONoti_t gtPMIONoti;
/*************************************************************************************************/
/*                                             Implementation                                    */
/* ***********************************************************************************************/
static void PMIO_Delay
(
    uint32_t                              uiCnt
)
{
    uint32_t uiDesc;

    uiDesc  = uiCnt;

    if(uiDesc == (uint32_t)(0UL))
    {
        while(TRUE)
        {
            BSP_NOP_DELAY();
        }
    }
    else
    {
        for(; uiDesc > (uint32_t)(0UL) ; uiDesc--)
        {
            BSP_NOP_DELAY();
        }
    }
}

static void PMIO_SetPort
(
    PMIOPortSel_t                       uiPortType,
    uint32_t                              uiPort32
)
{
    switch(uiPortType)
    {
        case PMIO_PORT_SEL_RELEASE:
            {
                //GPK port default is controled by PMGPIO
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32);  //0: pin controlled by PMGPIO
                PMIO_REG_PMGPIO_EN  &= ~(uiPort32);  //0: input
                PMIO_REG_PMGPIO_IEN &= ~(uiPort32);  //0: input disable
                break;
            }
        case PMIO_PORT_SEL_GPIO:
            {
                PMIO_REG_PMGPIO_FS  |=  (uiPort32); //1: pin controlled by GPIO
                PMIO_REG_PMGPIO_IEN |=  (uiPort32);  //1: input enable
                break;
            }
        case PMIO_PORT_SEL_PMGPIO_IN:
            {
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32);  //0: pin controlled by PMGPIO
                PMIO_REG_PMGPIO_EN  &= ~(uiPort32);  //0: input
                PMIO_REG_PMGPIO_IEN |=  (uiPort32);  //1: input enable
                break;
            }
        case PMIO_PORT_SEL_PMGPIO_OUT_L:
            {
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32); //0: pin controlled by PMGPIO_DAT/EN
                PMIO_REG_PMGPIO_EN  |=  (uiPort32); //1: output
                PMIO_REG_PMGPIO_DAT &= ~(uiPort32); //0: Low
                break;
            }
        case PMIO_PORT_SEL_PMGPIO_OUT_H:
            {
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32); //0: pin controlled by PMGPIO_DAT/EN
                PMIO_REG_PMGPIO_EN  |=  (uiPort32); //1: output
                PMIO_REG_PMGPIO_DAT |=  (uiPort32); //1: High
                break;
            }
        case PMIO_PORT_SEL_PMGPIO_PU:
            {
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32);  //0: pin controlled by PMGPIO_DAT/EN
                PMIO_REG_PMGPIO_PS  |=  (uiPort32);  //1: Set to pull-up
                PMIO_REG_PMGPIO_PE  |=  (uiPort32); //pull up/down En
                break;
            }
        case PMIO_PORT_SEL_PMGPIO_PD:
            {
                PMIO_REG_PMGPIO_FS  &= ~(uiPort32);  //0: pin controlled by PMGPIO_DAT/EN
                PMIO_REG_PMGPIO_PS  &= ~(uiPort32); //0: Set to pull-down
                PMIO_REG_PMGPIO_PE  |=  (uiPort32); //pull up/down En
                break;
            }
    }
}

static void PMIO_InterruptHandler
(
    void *                              pArg
)
{
    PMIOGpkIntList_t *sInt = (PMIOGpkIntList_t*)pArg;

    (void)pArg;

    PMIO_D("Detection power change.\n");

#ifdef PMIO_FEATURE_ACC_TIMER
    PMIO_SetTimer(0UL);
#endif

    /*GPK interrupt is handled by an external message handler. */
    if(PMIO_NotiGpkIrq(sInt->uiSrc, sInt->ucType) == SAL_RET_FAILED)
    {
        if(sInt->uiSrc == PMIO_VA_INTERRUPT_SRC_BU)
        {
            PMIO_D("irq: BU OFF\n");
            PMIO_PowerDown(PMIO_MODE_POWER_DEEP_DOWN);
        }

        if(sInt->uiSrc == PMIO_VA_INTERRUPT_SRC_ACC)
        {
            if(sInt->ucType == GIC_INT_TYPE_EDGE_FALLING)
            {
                PMIO_D("irq: ACC OFF\n");
                PMIO_PowerDown(PMIO_MODE_POWER_DOWN);
            }
            else
            {
                /*0: not-inverted(low->high)*/
                PMIO_D("irq: ACC ON\n");
            }
        }
    }
}

static void PMIO_SetInterruptHandler
(
    PMIOGpkIntList_t *sInt
)
{
    uint32_t uiReverseSrcOffset = 0UL;
    uint32_t uiDepthRegAdr = 0UL;
    uint32_t uiDepthRegVa = 0UL;

    if(sInt != NULL_PTR)
    {
        if(sInt->uiGpk > 0UL)
        {
            if(sInt->uiDepth > 0UL)
            {
                uiDepthRegAdr = MCU_BSP_GPIO_BASE + 0x800 + (0x4 * (sInt->uiSrc - GIC_EXT0));
                uiDepthRegVa = (1UL<<24UL);
                SAL_WriteReg(uiDepthRegVa, uiDepthRegAdr);
            }

            (void)GPIO_IntExtSet(sInt->uiSrc, sInt->uiGpk);

            if(sInt->ucType == GIC_INT_TYPE_EDGE_FALLING)
            {
                uiReverseSrcOffset += 10UL;
            }

            (void)GIC_IntVectSet( \
                    sInt->uiSrc + uiReverseSrcOffset, \
                    GIC_PRIORITY_NO_MEAN, \
                    sInt->ucType, \
                    &PMIO_InterruptHandler, \
                    (void*)sInt );

            (void)GIC_IntSrcEn(sInt->uiSrc + uiReverseSrcOffset);

            if(sInt->uiDepth > 0UL)
            {
                uiDepthRegVa |= (sInt->uiDepth & 0xFFFFFF);
                SAL_WriteReg(uiDepthRegVa, uiDepthRegAdr);
            }

            //mcu_printf("GIC SEt Complet\n");
        }
        else
        {
            (void)GIC_IntSrcDis(sInt->uiSrc);
        }
    }
    else
    {
        PMIO_E("Parameter is NULL \n");
    }

}

static void PMIO_SetGpkInterrupt
(
    uint32_t uiIntSrc,
    uint8  ucIntType,
    uint32_t uiGpk,
    uint32_t uiDepth
)
{
    static PMIOGpkIntList_t sIntList[8];
    uint32_t uiListIdx;
    uint8 ucWrongParam = 0U;

    if(ucIntType == 0U)
    {
        ucWrongParam = 1U;
    }
    if(uiGpk < GPIO_GPK(0UL))
    {
        ucWrongParam = 1U;
    }

    if(ucWrongParam == 0U)
    {
        if(uiIntSrc > PMIO_VA_INTERRUPT_SRC_ALL)
        {
            for(uiListIdx = 0UL; uiListIdx < 8UL ; uiListIdx++)
            {
                if(
                        (sIntList[uiListIdx].uiSrc == uiIntSrc) && \
                        (sIntList[uiListIdx].uiSrc == ucIntType) && \
                        (sIntList[uiListIdx].uiSrc == uiGpk)
                  )
                {
                    PMIO_E("Already interrupt in used. \n");
                    PMIO_E(" Registration failure (%d, %d, %d)\n", uiIntSrc, ucIntType, uiGpk);
                    break;
                }
            }

            if(uiListIdx == 8UL)
            {
                for(uiListIdx = 0UL; uiListIdx < 8UL ; uiListIdx++)
                {
                    if(sIntList[uiListIdx].uiSrc == 0UL)
                    {
                        sIntList[uiListIdx].uiSrc  = uiIntSrc;
                        sIntList[uiListIdx].ucType = ucIntType;
                        sIntList[uiListIdx].uiGpk  = uiGpk;
                        sIntList[uiListIdx].uiDepth  = uiDepth;
                        break;
                    }
                }

                if(uiListIdx < 8UL)
                {
                    PMIO_SetInterruptHandler(&(sIntList[uiListIdx]));
                }
                else
                {
                    PMIO_E("External Interrupt is full.\n");
                    PMIO_E(" Registration failure (%d, %d, %d)\n", uiIntSrc, ucIntType, uiGpk);
                }
            }

        }
        else
        {
            for(uiListIdx = 0UL; uiListIdx < 8UL ; uiListIdx++)
            {
                if(sIntList[uiListIdx].uiSrc != 0)
                {
                    sIntList[uiListIdx].ucType = 0U;
                    sIntList[uiListIdx].uiGpk  = 0UL;
                    sIntList[uiListIdx].uiDepth  = 0UL;
                    PMIO_SetInterruptHandler(&(sIntList[uiListIdx]));
                }
            }
        }
    }
}

static void PMIO_ReducePowerLeakageCurrent
(
    uint32_t uiExceptPort
)
{
    /* Power leakage current must be checked on user's hardware environment */
    /********************************************
    GPIO_K00    GPIO_K00
    GPIO_K01    BU_DET0
    GPIO_K02    GPIO_K02
    GPIO_K03    ACC_DET
    GPIO_K04    IGN_DET
    GPIO_K05    EMG_WAKE_IN
    GPIO_K06    CAN0_INH
    GPIO_K07    DOOR_OPEN_DET
    GPIO_K08    CAN0_STB
    GPIO_K09    LIN_TX
    GPIO_K10    LIN_RX
    GPIO_K11
    GPIO_K12    STR_MODE
    GPIO_K13    CAN0_TX
    GPIO_K14    CAN0_RX
    GPIO_K15    CAN1_TX
    GPIO_K16    CAN1_RX
    GPIO_K17    CAN2_TX
    GPIO_K18    CAN2_RX
    *********************************************/
    PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_IN, (PMIO_VA_LEAKAGE_IN_PORT & (~(uiExceptPort))) );

    PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_OUT_L, (PMIO_VA_LEAKAGE_OUT_L_PORT & (~(uiExceptPort))) );
    PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_OUT_H, (PMIO_VA_LEAKAGE_OUT_H_PORT & (~(uiExceptPort))) );

    PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_PU, PMIO_VA_LEAKAGE_PU_PORT);
    PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_PD, PMIO_VA_LEAKAGE_PD_PORT);

    PMIO_REG_PMGPIO_FS  = (PMIO_VA_BITCLEAR);  //0: pin controlled by PMGPIO
    PMIO_REG_PMGPIO_IEN = (PMIO_VA_BITALL);  //1: input enable
}

static void PMIO_SetTrimData
(
    uint32_t uiSwDt,
    uint32_t uiAwoDt
)
{
    PMIO_REG_SW_TRIM0 = uiSwDt;
    PMIO_REG_SW_TRIM1 = uiAwoDt;
    PMIO_REG_PMGPIO_CTL |= PMIO_VA_BIT28;
}

static void PMIO_EnTrimData
(
    uint32_t uiSwEn,
    uint32_t uiAwoEn
)
{
    if((uiSwEn == 1UL) || (uiAwoEn == 1UL))
    {
        PMIO_REG_PMGPIO_CTL |= PMIO_VA_BIT28;
    }
    else
    {
        PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT28);
    }

    if(uiSwEn == 1UL)
    {
        PMIO_REG_PMGPIO_CTL |= (PMIO_VA_BIT27 | PMIO_VA_BIT25);
    }
    else
    {
        PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT27 | PMIO_VA_BIT25);
    }

    if(uiAwoEn == 1UL)
    {
        PMIO_REG_PMGPIO_CTL |= (PMIO_VA_BIT26 | PMIO_VA_BIT24);
    }
    else
    {
        PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT26 | PMIO_VA_BIT24);
    }
}

/* **********************************************************************************************/
/*                                             EXTERN FUNCTION                                  */
/* **********************************************************************************************/

void PMIO_EarlyWakeUp
(
    void
)
{
    PMIOReason_t    tReason;
    PMIOMode_t      tMode;

    tReason     = PMIO_GetBootReason();
    tMode       = PMIO_GetBootMode();

    gReasonForDbg      = (uint32_t)tReason;
    gModeForDbg      = (uint32_t)tMode;

    PMIO_SetPower(PMIO_MODE_POWER_ON);

#ifdef PMIO_FEATURE_RTC_PMWKUP
    PMIO_SetRtcWakeUp(0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL);
#endif
}

void PMIO_PowerDown
(
    PMIOMode_t      tMode
)
{
    static uint8 uiRunPwdn = 0U;
#ifdef PMIO_FEATURE_LEAKAGE_REDUCTION_GPK_PORT
    uint32_t uiExcludingGpk = 0UL;
#endif

    if(uiRunPwdn == 0U)
    {
        uiRunPwdn = 1U;
        PMIO_D("power down(%d) start\n\n", tMode);
        PMIO_SetGpkInterrupt(PMIO_VA_INTERRUPT_SRC_ALL, 0U, 0UL, 0UL);

#ifdef PMIO_FEATURE_LEAKAGE_REDUCTION_GPK_PORT
        uiExcludingGpk = PMIO_VA_BU_DET_PORT
                            | PMIO_VA_ACC_DET_PORT
                            | PMIO_VA_CAN_DET_PORT
                            /*| PMIO_EXTN_GetUsingGpk32()*/;

        PMIO_ReducePowerLeakageCurrent(uiExcludingGpk);
#endif

        /*This bit must be set to “1” before core power (VDDI) is turned off.*/
        PMIO_REG_PMGPIO_CTL |=  (PMIO_VA_BIT0); //disable Latch/LVS

        /* set wake-up event */
        PMIO_SetPort(PMIO_PORT_SEL_PMGPIO_IN, (PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT|PMIO_VA_CAN_DET_PORT));

        PMIO_REG_PMGPIO_POL &= ~(PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT); //0:STR 1:STF
        PMIO_REG_PMGPIO_POL |=  (PMIO_VA_CAN_DET_PORT); //0:STR 1:STF
        PMIO_REG_PMGPIO_EE0 =   (PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT|PMIO_VA_CAN_DET_PORT); //0:Disable 1: En
        PMIO_REG_PMGPIO_EE1 =   (PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT|PMIO_VA_CAN_DET_PORT); //0:Disable 1: En

        if(tMode == PMIO_MODE_POWER_DOWN)
        {
#ifdef PMIO_FEATURE_RTC_PMWKUP
            PMIO_SetRtcWakeUp(
                    1UL,
                    1UL,
                    PMIO_VA_RTC_LIMIT_YEAR,
                    PMIO_VA_RTC_LIMIT_MON,
                    PMIO_VA_RTC_LIMIT_DAY,
                    PMIO_VA_RTC_LIMIT_DATE,
                    PMIO_VA_RTC_LIMIT_HOUR,
                    PMIO_VA_RTC_LIMIT_MIN,
                    PMIO_VA_RTC_LIMIT_SEC
                    );
#endif
        }
        else if(tMode == PMIO_MODE_POWER_DEEP_DOWN)
        {
#ifdef PMIO_FEATURE_RTC_PMWKUP
            PMIO_SetRtcWakeUp(0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL);
#endif
        }
        else
        {
            PMIO_D("Unknown Power mode %d\n\n", (uint32_t)tMode);
        }

        while(TRUE)
        {
            PMIO_REG_PMGPIO_STR =  (PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT); // Clear evnet status
            PMIO_REG_PMGPIO_STF =  (PMIO_VA_CAN_DET_PORT); // Clear evnet status
            PMIO_REG_PMGPIO_CTL |=  (PMIO_VA_BIT5|PMIO_VA_BIT4);

            PMIO_SetPower(tMode);
        }
    }
    else
    {
        PMIO_E("Already running Power Down\n\n");
    }
}

void PMIO_DebugPrintPowerStatus
(
    void
)
{
    PMIOReason_t tReason;
    PMIOMode_t tMode;

    tReason = (PMIOReason_t)gReasonForDbg;
    tMode   = (PMIOMode_t)gModeForDbg;


    switch(tMode)
    {
        case PMIO_MODE_NOT_SET :
        {
            PMIO_D("[PMIO: INFO ] PMIO_MODE_NOT_SET\n");
            break;
        }
        case PMIO_MODE_POWER_ON :
        {
            PMIO_D("[PMIO: INFO ] PMIO_MODE_POWER_ON\n");
            break;
        }
        case PMIO_MODE_POWER_DOWN :
        {
            PMIO_D("[PMIO: INFO ] PMIO_MODE_POWER_DOWN\n");
            break;
        }
        case PMIO_MODE_POWER_DEEP_DOWN :
        {
            PMIO_D("[PMIO: INFO ] PMIO_MODE_POWER_DEEP_DOWN\n");
            break;
        }
        default :
        {
            PMIO_D("[PMIO: INFO ] PMIO_MODE_UNKNOWN\n");
            break;
        }
    }

    switch(tReason)
    {
        case PMIO_REASON_NOT_SET :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_NOT_SET\n");
            break;
        }

        case PMIO_REASON_BU_ON :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_BU_ON\n");
            break;
        }

        case PMIO_REASON_ACC_ON :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_ACC_ON\n");
            break;
        }

        case PMIO_REASON_CAN_ON :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_CAN_ON\n");
            break;
        }

        case PMIO_REASON_RTC_ON :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_RTC_ON\n");
            break;
        }

        default :
        {
            PMIO_D("[PMIO: INFO ] PMIO_REASON_UNKNOWN\n");
            break;
        }
    }

#ifdef PMIO_FEATURE_ACC_TIMER
    PMIO_D("[PMIO: INFO ] Timer %d\n", gTimerForDbg);
#endif
}


void PMIO_Init
(
    void
)
{
#ifdef PMIO_FEATURE_USE_TRIM_CTRL
    uint32_t uiSwDt;
    uint32_t uiAwoDt;

    uiSwDt = 0UL;
    uiAwoDt = 0UL;
#endif

    ////Clear Event
    PMIO_REG_PMGPIO_STR = PMIO_VA_BITALL;
    PMIO_REG_PMGPIO_STF = PMIO_VA_BITALL;

#ifdef PMIO_FEATURE_USE_TRIM_CTRL
    #ifdef PMIO_FEATURE_USE_TRIM_CTRL_MANUAL
    uiSwDt  = PMIO_VA_TRIM_SW_DATA;
    uiAwoDt = PMIO_VA_TRIM_AWO_DATA;
    #else
    uiSwDt  = SAL_ReadReg(PMIO_VA_PMU_ECID_USER0_FBOUT0_ADDR) & (0x3FFFFFFFUL);
    uiAwoDt = (SAL_ReadReg(PMIO_VA_PMU_ECID_USER0_FBOUT1_ADDR) & (0x0FFFFFF8UL)) << 2UL;
    #endif
    PMIO_SetTrimData(uiSwDt, uiAwoDt);
    PMIO_EnTrimData(1UL,1UL);
#endif

    //Set PMGPIO port
    PMIO_SetPort(PMIO_PORT_SEL_GPIO, (PMIO_VA_ACC_DET_PORT|PMIO_VA_BU_DET_PORT));

    if( PMIO_VA_BU_DET_GPK > GPIO_GPK(0))
    {
        (void)GPIO_Config(PMIO_VA_BU_DET_GPK, GPIO_FUNC((uint32_t)0)|GPIO_INPUT|GPIO_INPUTBUF_EN);
    }

    if( PMIO_VA_ACC_DET_GPK > GPIO_GPK(0))
    {
        (void)GPIO_Config(PMIO_VA_ACC_DET_GPK, GPIO_FUNC((uint32_t)0)|GPIO_INPUT|GPIO_INPUTBUF_EN);

#ifdef PMIO_FEATURE_ACC_TIMER
        if (GPIO_Get(PMIO_VA_ACC_DET_GPK) == 0U )
        {
            PMIO_SetTimer(PMIO_FUNC_SEC_TO_USEC(PMIO_VA_ACC_OFF_TIME_LIMIT));
        }
#endif
    }

    PMIO_SetGpkInterrupt( \
            PMIO_VA_INTERRUPT_SRC_BU, \
            GIC_INT_TYPE_EDGE_FALLING, \
            PMIO_VA_BU_DET_GPK, \
            0UL
            );

    PMIO_SetGpkInterrupt( \
            PMIO_VA_INTERRUPT_SRC_ACC, \
            GIC_INT_TYPE_EDGE_FALLING, \
            PMIO_VA_ACC_DET_GPK, \
            10000000UL
            );

    PMIO_SetGpkInterrupt( \
            PMIO_VA_INTERRUPT_SRC_ACC, \
            GIC_INT_TYPE_EDGE_RISING, \
            PMIO_VA_ACC_DET_GPK, \
            0UL
            );
}

static void PMIO_SetPower
(
    PMIOMode_t       tMode
)
{
    switch(tMode)
    {
        case PMIO_MODE_NOT_SET:
            {
                PMIO_D("power mode not set\n");
                break;
            }
        case PMIO_MODE_POWER_ON:
            {
                PMIO_EnTrimData(1UL,1UL);

                //retention off, init off, power on
                PMIO_REG_SW_LDO10 = (0UL);
                break;
            }
        case PMIO_MODE_POWER_DOWN:
        case PMIO_MODE_POWER_DEEP_DOWN:
            {
                PMIO_EnTrimData(1UL,1UL);

                //retention off, init on, power off
                PMIO_REG_SW_LDO10 = (1UL);
                break;
            }
        default:
            {
                PMIO_D("Unknown Power Mode %d\n", tMode);
                break;
            }
    }
}

static PMIOMode_t PMIO_GetBootMode
(
    void
)
{
    PMIOMode_t  tMode;

    tMode = PMIO_MODE_POWER_ON;

    return tMode;
}

PMIOReason_t PMIO_GetBootReason
(
    void
)
{
    PMIOReason_t tBoot;

    tBoot   = PMIO_REASON_NOT_SET;

    if((PMIO_REG_LVSE_EN & PMIO_VA_BIT8 ) != 0UL)
    {
        tBoot = PMIO_REASON_RTC_ON;
    }
    else if((PMIO_REG_PMGPIO_STR & (uint32_t)PMIO_VA_BU_DET_PORT) == 0UL)
    {
        if((PMIO_REG_PMGPIO_STF & (uint32_t)PMIO_VA_BU_DET_PORT) == 0UL)
        {
            if((PMIO_REG_PMGPIO_STR & PMIO_VA_ACC_DET_PORT) != 0UL)
            {
                tBoot = PMIO_REASON_ACC_ON;
            }
            else if((PMIO_REG_PMGPIO_STF & PMIO_VA_ACC_DET_PORT) != 0UL)
            {
                tBoot = PMIO_REASON_ACC_ON;
            }
            else
            {
                if((PMIO_REG_PMGPIO_STR & PMIO_VA_CAN_DET_PORT) != 0UL)
                {
                    tBoot = PMIO_REASON_CAN_ON;
                }

                if((PMIO_REG_PMGPIO_STF & PMIO_VA_CAN_DET_PORT) != 0UL)
                {
                    tBoot = PMIO_REASON_CAN_ON;
                }
            }
        }
    }
    else
    {
        tBoot = PMIO_REASON_BU_ON;
    }

    if(tBoot == PMIO_REASON_NOT_SET)
    {
        tBoot = PMIO_REASON_BU_ON;
    }

    return tBoot;
}

void PMIO_SetGpk(uint32_t uiPort32)
{
    uint32_t uiComplexBit;

    uiComplexBit = (uiPort32 & (PMIO_VA_BIT14|PMIO_VA_BIT15|PMIO_VA_BIT16|PMIO_VA_BIT17));

    PMIO_SetPort(PMIO_PORT_SEL_GPIO, (uiPort32|(uiComplexBit << 4UL)));
}

#ifdef PMIO_FEATURE_RTC_PMWKUP
static void PMIO_SetRtcWakeUp
(
        uint32_t uiSwEn,
        uint32_t uiAwoEn,
        uint32_t uiYear,
        uint32_t uiMon,
        uint32_t uiDay,
        uint32_t uiDate,
        uint32_t uiHour,
        uint32_t uiMin,
        uint32_t uiSec
)
{

    if((uiSwEn|uiAwoEn) != 0UL)
    {
        if(uiSwEn != 0)
        {
            PMIO_REG_PMGPIO_CTL |= PMIO_VA_BIT7;
        }

        if(uiAwoEn != 0)
        {
            PMIO_REG_PMGPIO_CTL |= PMIO_VA_BIT6;
        }

        RTC_SetAlarm(uiYear, uiMon, uiDay, uiDate, uiHour, uiMin, uiSec, NULL);
        RTC_SetTime(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL);
        RTC_EnableAlarm(RTC_IRQ_PMWKUP);
    }
    else
    {
        PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT7|PMIO_VA_BIT6);

        RTC_SetAlarm(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, NULL);
        RTC_EnableAlarm(RTC_IRQ_DISABLE);
    }
}
#endif

#ifdef PMIO_FEATURE_ACC_TIMER
static void PMIO_SetTimer
(
    uint32_t                              uiUsec
)
{
    gTimerForDbg = uiUsec;

    //PMIO_D("pmio Set Timer %d\n", uiUsec);

    if(uiUsec != 0UL)
    {
        (void) TIMER_Enable((TIMERChannel_t)PMIO_VA_TIMER_CH, uiUsec, \
                &PMIO_TimeoutInterruptHandler, NULL);

        (void) GIC_IntSrcEn((uint32_t)GIC_TIMER_0 + PMIO_VA_TIMER_CH);
    }
    else
    {
        (void) TIMER_Disable((TIMERChannel_t)PMIO_VA_TIMER_CH);
    }
}

static sint32 PMIO_TimeoutInterruptHandler
(
    TIMERChannel_t                      iChannel,
    void *                              pArgs
)
{
    (void)pArgs;

    if((iChannel == (TIMERChannel_t)PMIO_VA_TIMER_CH))
    {
        (void) TIMER_Disable((TIMERChannel_t)PMIO_VA_TIMER_CH);
        PMIO_D("ACC is off for %ds.\n", PMIO_VA_ACC_OFF_TIME_LIMIT);
        PMIO_D("WAIT_ACC_ON is expired. Req the Power Deep Down.\n");
        PMIO_PowerDown(PMIO_MODE_POWER_DEEP_DOWN);
    }
    else
    {
        PMIO_D("TimerIntr fail null or mismatched ch %d\n", (uint32_t)iChannel);
    }
    return (sint32)SAL_RET_SUCCESS;
}
#endif

void PMIO_SetNotiGpkIrq
(
    uint32_t                              puiNotiIrqSrc,
    uint32_t                              puiNotiIrqSrcDirect,
    POWERAppHandler                     fHandler
)
{
    uint32_t uiIdx;
    uint32_t uiDiIdx;
    uint32_t uiEmptyIdx;

    uiEmptyIdx = 0xFFFFFFFFUL;

    if(fHandler != NULL)
    {
        PMIO_D("Set notification of gpk irq for application.\n");

        for(uiIdx = 0; uiIdx < PMIO_VA_MAX_EXT_NOTI_SIZE ; uiIdx++)
        {
#if 0
            mcu_printf("Idx %d, 0x%x, 0x%x, (0x%08x, 0x%08x, 0x%08x, 0x%08x)\n",
                    uiIdx,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrcDirect,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[0],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[1],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[2],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[3]
                    );
#endif
            if( (uiEmptyIdx == 0xFFFFFFFFUL) && (gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc == 0UL) )
            {
                uiEmptyIdx = uiIdx;
            }

            if(gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc == puiNotiIrqSrc)
            {
                break;
            }
        }

        if(uiIdx == PMIO_VA_MAX_EXT_NOTI_SIZE)
        {
            uiIdx = uiEmptyIdx;
        }

        gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc = puiNotiIrqSrc;
        gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrcDirect |= puiNotiIrqSrcDirect;

        for(uiDiIdx = 0UL ; uiDiIdx < 4UL/*GIC INT TYPE MAX SIZE*/ ; uiDiIdx++)
        {
            if( ((puiNotiIrqSrcDirect>>uiDiIdx) & 0x1UL) == 0x1UL)
            {
                gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[uiDiIdx] = fHandler;

                if(gtPMIONoti.uiTaskId == 0)
                {
                    if((SAL_TaskCreate(&(gtPMIONoti.uiTaskId), \
                                    (const uint8 *)"PMIO_NotiTask", \
                                    (SALTaskFunc)&PMIO_NotiTask, \
                                    &(gtPMIONoti.uiTaskStack[0]), \
                                    (uint32_t)2048, \
                                    SAL_PRIO_POWER_MANAGER, \
                                    NULL)) == SAL_RET_SUCCESS)
                    {
                        gtPMIONoti.uiEventNum = 1UL;

                        if ((SAL_EventCreate((uint32_t *)&(gtPMIONoti.uiEventId), \
                                        (const uint8 *)"msg signal event created", 0)) != SAL_RET_SUCCESS)
                        {
                            PMIO_E("stub create event FAIL.\n");
                        }
                    }
                    else
                    {
                        PMIO_E("stub pmio create task FAIL.\n");
                    }
                    //PMIO_D("Create Task for Noti gpk %d\n", gtPMIONoti.uiTaskId);
                }
            }
        }
#if 0
        mcu_printf("Set Idx %d, 0x%x, 0x%x, (0x%08x, 0x%08x, 0x%08x, 0x%08x)\n",
                uiIdx,
                gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc,
                gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrcDirect,
                gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[0],
                gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[1],
                gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[2],
                gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[3]
                );
#endif
    }
}

static SALRetCode_t PMIO_NotiGpkIrq
(
    uint32_t                              uiGicSrc,
    uint32_t                              uiGicDirect
)
{
    SALRetCode_t tRet;
    uint32_t uiIdx;
    uint32_t uiDiIdx;

    tRet = SAL_RET_FAILED;


    if(gtPMIONoti.uiTaskId != 0)
    {
        for(uiIdx = 0UL; uiIdx < 10UL ; uiIdx++)
        {
#if 0
            mcu_printf("uiIdx %d, 0x%x, 0x%x, (0x%08x, 0x%08x, 0x%08x, 0x%08x)\n",
                    uiIdx,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrcDirect,
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[0],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[1],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[2],
                    gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[3]
                    );
#endif
            if(gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrc == uiGicSrc)
            {
                if( (gtPMIONoti.tPMIONotiIntList[uiIdx].uiSrcDirect & uiGicDirect) != 0UL )
                {
                    for(uiDiIdx = 0UL ; uiDiIdx < 4UL/*GIC INT TYPE MAX SIZE*/ ; uiDiIdx++)
                    {
                        if( ((uiGicDirect >> uiDiIdx) & 0x1UL) == 0x1UL)
                        {
                            tRet = SAL_RET_SUCCESS;

                            gtPMIONoti.fSelHandler = gtPMIONoti.tPMIONotiIntList[uiIdx].fHandler[uiDiIdx];

                            (void)SAL_EventSet(
                                    gtPMIONoti.uiEventId,
                                    gtPMIONoti.uiEventNum,
                                    SAL_EVENT_OPT_FLAG_SET
                                    );
                        }
                    }
                    break;
                }
            }
        }
    }
#if 0
    mcu_printf("select Idx %d, handler 0x%08x\n",uiIdx, fHandler);
#endif
    return tRet;
}

static void PMIO_NotiTask
(
    void *                              pArg
)
{
    uint32_t uiFlag;

    uiFlag = 0UL;

    while(TRUE)
    {
        (void)SAL_EventGet(gtPMIONoti.uiEventId, gtPMIONoti.uiEventNum, 0UL, \
                (((uint32_t) SAL_EVENT_OPT_SET_ANY)|((uint32_t) SAL_OPT_BLOCKING)), &uiFlag);

        if(uiFlag > 0UL)
        {
            if(gtPMIONoti.fSelHandler != NULL)
            {
                gtPMIONoti.fSelHandler();
            }
            else
            {
                PMIO_E("Irq Noti Handler is null\n");
            }

            gtPMIONoti.fSelHandler = NULL;
        }
        else
        {
            PMIO_D("Irq Noti event zero\n");
        }

        (void)SAL_EventSet(gtPMIONoti.uiEventId, gtPMIONoti.uiEventNum, SAL_EVENT_OPT_CLR_ALL);
    }
}

#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )

