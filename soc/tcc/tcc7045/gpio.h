/*
***************************************************************************************************
*
*   FileName : gpio.h
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

#ifndef MCU_BSP_GPIO_HEADER
#define MCU_BSP_GPIO_HEADER

#include <sal_internal.h>

/*
***************************************************************************************************
*                                            GLOBAL DEFINITIONS
***************************************************************************************************
*/

#if (DEBUG_ENABLE)
    #include "debug.h"

    #define GPIO_D(fmt,args...)         {LOGD(DBG_TAG_GPIO, fmt, ## args)}
    #define GPIO_E(fmt,args...)         {LOGE(DBG_TAG_GPIO, fmt, ## args)}
#else
    #define GPIO_D(fmt,args...)
    #define GPIO_E(fmt,args...)
#endif


/*
 * gpio cfg structures
 *   [31:14]: reserved
 *   [11:10]: input buffer
 *   [9]    : direction
 *   [8:6]  : driver strength (0~3)
 *   [5:4]  : pull up/down
 *   [3:0]  : function selection (0~15)
 */

#define GPIO_INPUTBUF_SHIFT             (10)
#define GPIO_INPUTBUF_MASK              (0x3UL)
#define GPIO_INPUTBUF_EN                ((2UL | 1UL) << (uint32)GPIO_INPUTBUF_SHIFT)
#define GPIO_INPUTBUF_DIS               ((2UL | 0UL) << (uint32)GPIO_INPUTBUF_SHIFT)

#define GPIO_OUTPUT_SHIFT               (9)
#define GPIO_OUTPUT                     (1UL << (uint32)GPIO_OUTPUT_SHIFT)
#define GPIO_INPUT                      (0UL << (uint32)GPIO_OUTPUT_SHIFT)

#define GPIO_DS_SHIFT                   (6)
#define GPIO_DS_MASK                    (0x7UL)
#define GPIO_DS(x)                      ((((x) & (uint32)GPIO_DS_MASK) | 0x4UL) << (uint32)GPIO_DS_SHIFT)

#define GPIO_PULL_SHIFT                 (4)
#define GPIO_PULL_MASK                  (0x3UL)
#define GPIO_NOPULL                     (0UL << (uint32)GPIO_PULL_SHIFT)
#define GPIO_PULLUP                     (1UL << (uint32)GPIO_PULL_SHIFT)
#define GPIO_PULLDN                     (2UL << (uint32)GPIO_PULL_SHIFT)

#define GPIO_FUNC_MASK                  (0xFUL)
#define GPIO_FUNC(x)                    ((x) & (uint32)GPIO_FUNC_MASK)

#define GPIO_MFIO_CFG_CH_SEL0           (0)
#define GPIO_MFIO_CFG_PERI_SEL0         (4)
#define GPIO_MFIO_CFG_CH_SEL1           (8)
#define GPIO_MFIO_CFG_PERI_SEL1         (12)
#define GPIO_MFIO_CFG_CH_SEL2           (16)
#define GPIO_MFIO_CFG_PERI_SEL2         (20)
#define GPIO_MFIO_DISABLE               (0)
#define GPIO_MFIO_SPI2                  (1)
#define GPIO_MFIO_UART3                 (2)
#define GPIO_MFIO_I2C3                  (3)
#define GPIO_MFIO_SPI3                  (1)
#define GPIO_MFIO_UART4                 (2)
#define GPIO_MFIO_I2C4                  (3)
#define GPIO_MFIO_SPI4                  (1)
#define GPIO_MFIO_UART5                 (2)
#define GPIO_MFIO_I2C5                  (3)
#define GPIO_MFIO_CH0                   (0)
#define GPIO_MFIO_CH1                   (1)
#define GPIO_MFIO_CH2                   (2)
#define GPIO_MFIO_CH3                   (3)
#define GPIO_PERICH_SEL_UARTSEL_0       (0)
#define GPIO_PERICH_SEL_UARTSEL_1       (1)
#define GPIO_PERICH_SEL_UARTSEL_2       (2)
#define GPIO_PERICH_SEL_I2CSEL_0        (3)
#define GPIO_PERICH_SEL_I2CSEL_1        (4)
#define GPIO_PERICH_SEL_I2CSEL_2        (5)
#define GPIO_PERICH_SEL_SPISEL_0        (6)
#define GPIO_PERICH_SEL_SPISEL_1        (7)
#define GPIO_PERICH_SEL_I2SSEL_0        (8)
#define GPIO_PERICH_SEL_PWMSEL_0        (10)
#define GPIO_PERICH_SEL_PWMSEL_1        (12)
#define GPIO_PERICH_SEL_PWMSEL_2        (14)
#define GPIO_PERICH_SEL_PWMSEL_3        (16)
#define GPIO_PERICH_SEL_PWMSEL_4        (18)
#define GPIO_PERICH_SEL_PWMSEL_5        (20)
#define GPIO_PERICH_SEL_PWMSEL_6        (22)
#define GPIO_PERICH_SEL_PWMSEL_7        (24)
#define GPIO_PERICH_SEL_PWMSEL_8        (26)
#define GPIO_PERICH_CH0                 (0)
#define GPIO_PERICH_CH1                 (1)
#define GPIO_PERICH_CH2                 (2)
#define GPIO_PERICH_CH3                 (3)

/*
 * gpio port & pin structures
 *   [31:10]: reserved
 *   [9:5] : port (A,B,C,...)
 *   [4:0] : pin number (0~31)
 */

#define GPIO_PIN_MASK                   (0x1FUL)
#define GPIO_PIN_NUM_MASK               (0x3FUL) // original 1FUL , avoid code sonar warning


#define GPIO_PORT_SHIFT                 (5)
#define GPIO_PORT_MASK                  ((uint32)0x1F << (uint32)GPIO_PORT_SHIFT)

//(n<<GPIO_PORT_SHIFT)                  n = ofset/0x40
#define GPIO_PORT_A                     ((uint32)0 << (uint32)GPIO_PORT_SHIFT)        // offset: 0x000
#define GPIO_PORT_B                     ((uint32)1 << (uint32)GPIO_PORT_SHIFT)        // offset: 0x040
#define GPIO_PORT_C                     ((uint32)2 << (uint32)GPIO_PORT_SHIFT)        // offset: 0x080
#define GPIO_PORT_K                     ((uint32)3<<(uint32)GPIO_PORT_SHIFT)        // offset: 0x0c0

#define GPIO_GPA(x)                     (GPIO_PORT_A | ((x) & (uint32)0x1F))
#define GPIO_GPB(x)                     (GPIO_PORT_B | ((x) & (uint32)0x1F))
#define GPIO_GPC(x)                     (GPIO_PORT_C | ((x) & (uint32)0x1F))
#define GPIO_GPK(x)                     (GPIO_PORT_K | ((x) & (uint32)0x1F))


#define GPIO_GP_MAX                     GPIO_GPK((uint32)0x1f)


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                          GPIO_Config
*
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @param    [In] uiConfig   :   Gpio configuration options
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_Config
(
    uint32                              uiPort,
    uint32                              uiConfig
);

/*
***************************************************************************************************
*                                          GPIO_Get
*
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @return   Gpio data value (0:Low or 1:High)
*
* Notes
*
***************************************************************************************************
*/
uint8 GPIO_Get
(
    uint32                              uiPort
);

/*
***************************************************************************************************
*                                          GPIO_Set
*
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @param    [In] uiData     :   Gpio data value, (0 or 1)
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_Set
(
    uint32                              uiPort,
    uint32                              uiData
);

/*
***************************************************************************************************
*                                          GPIO_ToNum
*
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @return
*
* Notes
*
***************************************************************************************************
*/
uint32 GPIO_ToNum
(
    uint32                              uiPort
);

/*
***************************************************************************************************
*                                          GPIO_PerichSel
*
* @param    [In] uiPerichSel    : Gpio peri select index
* @param    [In] uiCh           : Gpio peri select channel index
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_PerichSel
(
    uint32                              uiPerichSel,
    uint32                              uiCh
);


/*
***************************************************************************************************
*                                          GPIO_MfioCfg
*
* @param    [In] uiPeriSel      :   MFIO peri select index
* @param    [In] uiPeriType     :   MFIO peri select type (Disable/GPSB/UART/I2C)
* @param    [In] uiChSel        :   MFIO channel select index
* @param    [In] uiChNum        :   MFIO channel select value
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_MfioCfg
(
    uint32                              uiPeriSel,
    uint32                              uiPeriType,
    uint32                              uiChSel,
    uint32                              uiChNum
);

/*
***************************************************************************************************
*                                       GPIO_IntExtSet
*
* Configure external interrupt source select.
*
* @param    [In] uiIntId        : Index of Interrupt Source id.
* @param    [In] uiGpio         : Gpio port index, GPIO_GPX(X)
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/


SALRetCode_t GPIO_IntExtSet
(
    uint32                               uiIntId,
    uint32                               uiGpio
);

#endif  // MCU_BSP_GPIO_HEADER

