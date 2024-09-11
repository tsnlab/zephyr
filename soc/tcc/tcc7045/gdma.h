/*
***************************************************************************************************
*
*   FileName : gdma.h
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

#ifndef MCU_BSP_GDMA_HEADER
#define MCU_BSP_GDMA_HEADER

#if 1 //( MCU_BSP_SUPPORT_DRIVER_GDMA == 1 )

#include <sal_internal.h>

#define GDMA_CON_MAX                     (8UL)
#define GDMA_CH_MAX                     (2UL)


// UDMA address starts from 0xC003E000 and occupies 0x3FF for each of tx and rx.
// UDMA has 4 channels whose range of address is from 0xC003E000 to 0xC003FFFF(8KB).
// Valid address range of SRAM(micom) is only defined from 0xC0000000 to 0xC003FFFF(256KB).
// If it is needed to increase DMA buffer area in SRAM, refer to 4.6 Use fixed DMA buffer
// in TCC805x MICOM RTOS SDK-User Guide
#define GDMA_BUFF_SIZE                  (0x3ffUL) // 1023.
#define GDMA_BUFF_MARGIN                (0x0UL)

#define GDMA_CH0                        (0UL)
#define GDMA_CH1                        (1UL)

#define GDMA_INC                        (1U)
#define GDMA_NO_INC                     (0U)

#define GDMA_TRANSFER_SIZE_BYTE         (0U)
#define GDMA_TRANSFER_SIZE_HALF         (1U)
#define GDMA_TRANSFER_SIZE_WORD         (2U)

#define GDMA_MAX_LLI_NUM                (64UL)

#define GDMA_BURST_SIZE_1               (0U)
#define GDMA_BURST_SIZE_4               (1U)
#define GDMA_BURST_SIZE_8               (2U)
#define GDMA_BURST_SIZE_16              (3U)
#define GDMA_BURST_SIZE_32              (4U)
#define GDMA_BURST_SIZE_64              (5U)
#define GDMA_BURST_SIZE_128             (6U)
#define GDMA_BURST_SIZE_256             (7U)

#define GDMA_FLOW_TYPE_M2M              (0UL)
#define GDMA_FLOW_TYPE_M2P              (1UL)
#define GDMA_FLOW_TYPE_P2M              (2UL)
#define GDMA_FLOW_TYPE_P2P              (3UL)
#define GDMA_FLOW_TYPE_P2P_BY_DP        (4UL)
#define GDMA_FLOW_TYPE_M2P_BY_P         (5UL)
#define GDMA_FLOW_TYPE_P2M_BY_P         (6UL)
#define GDMA_FLOW_TYPE_P2P_BY_SP        (7UL)

#define GDMA_PERI_REQ_PORT_UART0_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART0_TX  (1UL)

#define GDMA_PERI_REQ_PORT_UART1_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART1_TX  (1UL)

#define GDMA_PERI_REQ_PORT_UART2_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART2_TX  (1UL)

#define GDMA_PERI_REQ_PORT_UART3_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART3_TX  (1UL)

#define GDMA_PERI_REQ_PORT_UART4_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART4_TX  (1UL)

#define GDMA_PERI_REQ_PORT_UART5_RX  (0UL)
#define GDMA_PERI_REQ_PORT_UART5_TX  (1UL)

#define GDMA_PERI_REQ_PORT_ICTC0  (2UL)
#define GDMA_PERI_REQ_PORT_ICTC1  (3UL)
#define GDMA_PERI_REQ_PORT_ICTC2  (4UL)
#define GDMA_PERI_REQ_PORT_ICTC3  (5UL)
#define GDMA_PERI_REQ_PORT_ICTC4  (6UL)
#define GDMA_PERI_REQ_PORT_ICTC5  (7UL)

#define GDMA_PERI_REQ_PORT_ADC0  (8UL)
#define GDMA_PERI_REQ_PORT_ADC1  (9UL)

#define GDMA_PERI_REQ_PORT_CAN0  (10UL)
#define GDMA_PERI_REQ_PORT_CAN1  (11UL)
#define GDMA_PERI_REQ_PORT_CAN2  (12UL)

// DMA Request Peripheral
enum
{
    GDMA_PERI_RX = 0,
    GDMA_PERI_TX = 1
};

// parameter :  pointer of GDMAInformation_t
// return : void
typedef void (*GDMAIsrCallBackFunc)
(
    void *                              pArg
);

typedef struct GDMA_INFORMATION
{
    uint32                              iCon;
    uint32                              iCh;
    uint8 *                             iSrcAddr;
    uint8 *                             iDestAddr;
    uint32                              iBufSize;
    uint32                              iTransSize;
    GDMAIsrCallBackFunc                 fpIsrCallbackForComplete;
} GDMAInformation_t;

/*
***************************************************************************************************
*                                         EXTERN FUNCTIONS
***************************************************************************************************
*/

sint32 GDMA_Init
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiIntPrio
);

void GDMA_ChannelEnable
(
    GDMAInformation_t *                 sDmacon
);

void GDMA_ChannelDisable
(
    GDMAInformation_t *                 sDmacon
);

void GDMA_SetFlowControl
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiFlow
);

void GDMA_SetAddrIncrement
(
    GDMAInformation_t *                 sDmacon,
    uint8                               ucDestInc,
    uint32                              uiSrcInc
);

void GDMA_SetTransferWidth
(
    GDMAInformation_t *                 sDmacon,
    uint8                               ucDestWidth,
    uint32                              uiSrcWidth
);

void GDMA_SetBurstSize
(
    GDMAInformation_t *                 sDmacon,
    uint8                               ucDestBurst,
    uint32                              uiSrcBurst
);

void GDMA_SetTransferSize
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiTransferSize
);

void GDMA_InterruptEnable
(
    GDMAInformation_t *                 sDmacon
);

void GDMA_WaitInterrupt
(
    GDMAInformation_t *                 sDmacon
);

void GDMA_InterruptDisable
(
    GDMAInformation_t *                 sDmacon
);

SALRetCode_t GDMA_Reset
(
    uint8                               ucCh
);

void GDMA_SetPeri
(
    GDMAInformation_t *                 sDmacon,
    uint8                               ucDestPeri,
    uint8                               ucSrcPeri
);

void GDMA_SetSrcAddr
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiAddr
);

void GDMA_SetDestAddr
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiAddr
);

void GDMA_SetlliAddr
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiLLIAddr
);


void GDMA_TransferMem2Mem
(
    GDMAInformation_t *                 sDmacon,
    uint32                              uiSrcAddr,
    uint32                              uiDestAddr,
    uint32                              uiLen
);

uint32 GDMA_GetTransferSize
(
    GDMAInformation_t *                 sDmacon
);

#endif  // ( MCU_BSP_SUPPORT_DRIVER_GDMA == 1 )

#endif  // MCU_BSP_GDMA_HEADER

