/*
***************************************************************************************************
*
*   FileName : uart_drv.h
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

#ifndef MCU_BSP_UART_DRV_HEADER
#define MCU_BSP_UART_DRV_HEADER

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/

#include <gic.h>
#include "gdma.h"

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

// UART Settings
#define UART_BUFF_SIZE                  (0x100UL)   // 256

#define UART_MODE_TX                    (0UL)
#define UART_MODE_RX                    (1UL)

#define UART_PORT_CFG_MAX               (16U)
#define UART_PORT_TBL_SIZE              (UART_PORT_CFG_MAX)

// GDMA Address
#define GDMA_ADDRESS_OFFSET             (0UL)
#define GDMA_ADDRESS_UNIT               (GDMA_BUFF_SIZE + GDMA_BUFF_MARGIN)
#define GDMA_ADDRESS_UNIT_CH_RX(ch)     ((uint32)GDMA_ADDRESS_OFFSET + ((GDMA_ADDRESS_UNIT * 2UL) * (ch)))
#define GDMA_ADDRESS_UNIT_CH_TX(ch)     (((uint32)GDMA_ADDRESS_OFFSET + ((GDMA_ADDRESS_UNIT * 2UL) * (ch))) + GDMA_ADDRESS_UNIT)

// channel number : 0, 1, 2, 3
// baudrate : 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600 etc.
// flow control. Enable : 1, Disable : 0
// mode : UART_POLLING_MODE, UART_INTR_MODE, UART_DMA_MODE

#define TCC_GPNONE                      (0xFFFFUL)
#define TCC_GPSD0(x)                    (TCC_GPNONE)
#define TCC_GPSD1(x)                    (TCC_GPNONE)
#define TCC_GPSD2(x)                    (TCC_GPNONE)

// UART FIFO buffer size
#define UART_TX_FIFO_SIZE               (8UL)
#define UART_RX_FIFO_SIZE               (12UL)

// UART Base address
#define UART_GET_BASE(n)                (MCU_BSP_UART_BASE + (0x10000UL * (n)))

// Raw Interrupt Status Register (RIS) Fields
// Masked Interrupt Status Register (MIS) Fields
// Interrupt Clear Register (ICR) Fields
#define UART_INT_OEIS                   (1UL << 10U)   // Overrun error interrupt
#define UART_INT_BEIS                   (1UL << 9U)    // Break error interrupt
#define UART_INT_PEIS                   (1UL << 8U)    // Parity error interrupt
#define UART_INT_FEIS                   (1UL << 7U)    // Framing error interrupt
#define UART_INT_RTIS                   (1UL << 6U)    // Receive timeout interrupt
#define UART_INT_TXIS                   (1UL << 5U)    // Transmit interrupt
#define UART_INT_RXIS                   (1UL << 4U)    // Receive interrupt

// DMA Control Register (DMACR) Fields
#define UART_DMACR_DMAONERR             (1UL << 2U)    // DMA on error
#define UART_DMACR_TXDMAE               (1UL << 1U)    // Transmit DMA enable
#define UART_DMACR_RXDMAE               (1UL << 0U)    // Receive DMA enable

/*
***************************************************************************************************
*                                         ENUMERATIONS
***************************************************************************************************
*/

typedef enum UART_WORD_LEN
{
    WORD_LEN_5 = 0,
    WORD_LEN_6,
    WORD_LEN_7,
    WORD_LEN_8
} UartWordLen_t;

typedef enum UART_PARITY
{
    PARITY_SPACE = 0,
    PARITY_EVEN,
    PARITY_ODD,
    PARITY_MARK
} UartParity_t;

/*
***************************************************************************************************
*                                         STRUCTS
***************************************************************************************************
*/

typedef struct UART_PARAM
{
    uint8                               sCh;
    uint32                              sPriority;     // Interrupt priority
    uint32                              sBaudrate;     // Baudrate
    uint8                               sMode;         // polling or interrupt or dma
    uint8                               sCtsRts;       // on/off
    uint8                               sPortCfg;      // port selection
    uint8                               sFIFO;         // on/off
    uint8                               s2StopBit;     // on/off
    UartWordLen_t                       sWordLength;   // 5~8 bits
    UartParity_t                        sParity;       // space, even, odd, mark
    GICIsrFunc                          sFnCallback;   // callback function
} UartParam_t;

// UART Interrupt mode
typedef struct UART_INTERRUPT_DATA
{
    uint8 *                             iXmitBuf;
    sint32                              iHead;
    sint32                              iTail;
    sint32                              iSize;
} UartInterruptData_t;

typedef struct UART_BOARD_PORT
{
    uint32                              bPortCfg;   // Config port ID
    uint32                              bPortTx;    // UT_TXD GPIO
    uint32                              bPortRx;    // UT_RXD GPIO
    uint32                              bPortRts;   // UT_RTS GPIO
    uint32                              bPortCts;   // UT_CTS GPIO
    uint32                              bPortFs;    // UART function select
    uint32                              bPortCH;    // Channel
} UartBoardPort_t;

typedef struct UART_STATUS
{
    boolean                             sIsProbed;
    uint32                              sBase;          // UART Controller base address
    uint8                               sCh;            // UART Channel
    uint8                               sOpMode;        // Operation Mode
    uint8                               sCtsRts;        // CTS and RTS
    uint8                               s2StopBit;      // 1: two stop bits are transmitted
    UartParity_t                        sParity;        // 0:disable, 1:enable
    UartWordLen_t                       sWordLength;    // Word Length
    UartBoardPort_t                     sPort;          // GPIO Port Infomation
    GDMAInformation_t                   sRxDma;         // Rx DMA
    GDMAInformation_t                   sTxDma;         // Tx DMA
    UartInterruptData_t                 sRxIntr;        // Rx Interrupt
    UartInterruptData_t                 sTxIntr;        // Tx Interrupt
} UartStatus_t;

#endif  // MCU_BSP_UART_DRV_HEADER

