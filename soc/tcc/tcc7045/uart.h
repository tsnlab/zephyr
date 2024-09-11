/*
***************************************************************************************************
*
*   FileName : uart.h
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

#ifndef MCU_BSP_UART_HEADER
#define MCU_BSP_UART_HEADER

#if 1//( MCU_BSP_SUPPORT_DRIVER_UART == 1 )

#if 0 // ( MCU_BSP_SUPPORT_DRIVER_GDMA != 1 )
    #error MCU_BSP_SUPPORT_DRIVER_GDMA value must be 1.
#endif  //

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#include <sal_internal.h>
#include <uart_drv.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

#ifndef ON
#define ON                              (TRUE)
#define OFF                             (FALSE)
#endif

// UART Channels
#define UART_CH0                        (0U)
#define UART_CH1                        (1U)
#define UART_CH2                        (2U)
#define UART_CH3                        (3U)
#define UART_CH4                        (4U)
#define UART_CH5                        (5U)
#define UART_CH_MAX                     (6U)

#define UART_POLLING_MODE               (0U)
#define UART_INTR_MODE                  (1U)
#define UART_DMA_MODE                   (2U)

#define UART_CTSRTS_ON                  (1U)
#define UART_CTSRTS_OFF                 (0U)

#define ENABLE_FIFO                     (1U)
#define DISABLE_FIFO                    (0U)

#define TWO_STOP_BIT_ON                 (1U)
#define TWO_STOP_BIT_OFF                (0U)

/*
***************************************************************************************************
*                                         UART DEFAULT SETTING
***************************************************************************************************
*/

// Set for using UART0 channel
#define UART_DEBUG_CH                   (UART_CH1)
#define UART_DEBUG_CLK                  (48000000UL)    // 48MHz

/*
***************************************************************************************************
*                                         REGISTERS
***************************************************************************************************
*/

// UART Register (BASE Address + Offset)
#define UART_REG_DR                     (0x00U)    // Data register
#define UART_REG_RSR                    (0x04U)    // Receive Status register
#define UART_REG_ECR                    (0x04U)    // Error Clear register
#define UART_REG_FR                     (0x18U)    // Flag register
#define UART_REG_IBRD                   (0x24U)    // Integer Baud rate register
#define UART_REG_FBRD                   (0x28U)    // Fractional Baud rate register
#define UART_REG_LCRH                   (0x2cU)    // Line Control register
#define UART_REG_CR                     (0x30U)    // Control register
#define UART_REG_IFLS                   (0x34U)    // Interrupt FIFO Level status register
#define UART_REG_IMSC                   (0x38U)    // Interrupt Mask Set/Clear register
#define UART_REG_RIS                    (0x3cU)    // Raw Interrupt Status register
#define UART_REG_MIS                    (0x40U)    // Masked Interrupt Status register
#define UART_REG_ICR                    (0x44U)    // Interrupt Clear register
#define UART_REG_DMACR                  (0x48U)    // DMA Control register

// UART Flag Register(FR) Fields
#define UART_FR_TXFE                    (1UL << 7U)    // Transmit FIFO empty
#define UART_FR_RXFF                    (1UL << 6U)    // Receive FIFO full
#define UART_FR_TXFF                    (1UL << 5U)    // Transmit FIFO full
#define UART_FR_RXFE                    (1UL << 4U)    // Receive FIFO empty
#define UART_FR_BUSY                    (1UL << 3U)    // UART busy
#define UART_FR_CTS                     (1UL << 0U)    // Clear to send

// UART Line Control Register (LCR_H) Fields
#define UART_LCRH_SPS                   (1UL << 7U)    // Stick parity select
#define UART_LCRH_WLEN(x)               ((x) << 5U)    // Word length
#define UART_LCRH_FEN                   (1UL << 4U)    // Enable FIFOs
#define UART_LCRH_STP2                  (1UL << 3U)    // Two stop bits select
#define UART_LCRH_EPS                   (1UL << 2U)    // Even parity select
#define UART_LCRH_PEN                   (1UL << 1U)    // Parity enable
#define UART_LCRH_BRK                   (1UL << 0U)    // Send break

// UART Control Register (CR) Fields
#define UART_CR_CTSEN                   (1UL << 15U)   // CTS hardware flow control enable
#define UART_CR_RTSEN                   (1UL << 14U)   // RTS hardware flow control enable
#define UART_CR_RTS                     (1UL << 11U)   // Request to send
#define UART_CR_RXE                     (1UL << 9U)    // Receive enable
#define UART_CR_TXE                     (1UL << 8U)    // Transmit enable
#define UART_CR_LBE                     (1UL << 7U)    // Loopback enable
#define UART_CR_EN                      (1UL << 0U)    // UART enable

/*
***************************************************************************************************
*                                       FUNCTION PROTOTYPES
***************************************************************************************************
*/

/******************************************************************************
* Function name:  UART_Open
* Description:    Open UART channel
* Parameter:      pUartCfg : UART configuration table
* Return value:   sint32
* Remarks:        Public API.
* Requirements:   None.
******************************************************************************/
sint32 UART_Open
(
    UartParam_t                         * pUartCfg
);

void UART_Close
(
    uint8                              ucCh
);

sint32 UART_Read
(
    uint8                               ucCh,
    uint8 *                             pucBuf,
    uint32                              uiSize
);

sint32 UART_Write
(
    uint8                               ucCh,
    const uint8 *                       pucBuf,
    uint32                              uiSize
);

uint32 UART_GetData
(
    uint8                               ucCh,
    sint32                              iWait,
    sint8 *                             pcErr
);

sint32 UART_GetChar
(
    uint8                               ucCh,
    sint32                              iWait,
    sint8 *                             pcErr
);

sint32 UART_PutChar
(
    uint8                               ucCh,
    uint8                               ucChar
);

void UART_SetLineControlReg
(
    uint8                               ucCh,
    uint32                              uiBits,
    uint8                               uiEnabled
);

void UART_SetInterruptClearReg
(
    uint8                               ucCh,
    uint32                              uiSetValue
);

void UART_SetErrorClearReg
(
    uint8                               ucCh,
    uint32                              uiSetValue
);

uint32 UART_GetReceiveStatusReg
(
    uint8                               ucCh
);

uint32 UART_GetRawInterruptStatusReg
(
    uint8                               ucCh
);

void UART_Init
(
    void
);

void UART_ISR
(
    void *                              pArg
);

#endif  // ( MCU_BSP_SUPPORT_DRIVER_UART == 1 )

#endif  // MCU_BSP_UART_HEADER

