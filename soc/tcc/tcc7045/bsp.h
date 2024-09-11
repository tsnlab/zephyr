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

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
// For Dolphin
#define TCC802x_BD_VER_0_1              (1)
#define TCC802x_BD_VER_1_0              (2)
#define TCC802x_BD_VER_2_0              (3)

// For Dolphin +
#define TCC803x_BD_VER_0_1              (30)  /** number start from 30 for Dolphin+  */
#define TCC803x_BD_VER_0_3              (31)  /** EVB 0.3                            */

#define TCC8059_BD_VER_0_1              (50)  /** Dolphin+ QD EVB 0.1                */
#define TCC8050_BD_VER_0_1              (51)  /** Dolphin3 E/M EVB 0.1               */

#if defined (MCU_BSP_BOARD_VER_TCC8059_EVM_0_1)
    #define TCC_EVM_BD_VERSION          (TCC8059_BD_VER_0_1)
#elif defined (MCU_BSP_BOARD_VER_TCC8050_EVM_0_1)
    #define TCC_EVM_BD_VERSION          (TCC8050_BD_VER_0_1)
#endif

#define BSP_CHIPSET_NAME_REG            (0x1B9361CCUL)
#define BSP_SIGNATURE_TCC8030           (0x18030000UL)
#define BSP_SIGNATURE_TCC8050           (0x18050000UL)
#define BSP_SIGNATURE_TCC8059           (0x18059000UL)

#define BSP_BIT_31                      (0x80000000UL)
#define BSP_BIT_30                      (0x40000000UL)
#define BSP_BIT_29                      (0x20000000UL)
#define BSP_BIT_28                      (0x10000000UL)
#define BSP_BIT_27                      (0x08000000UL)
#define BSP_BIT_26                      (0x04000000UL)
#define BSP_BIT_25                      (0x02000000UL)
#define BSP_BIT_24                      (0x01000000UL)
#define BSP_BIT_23                      (0x00800000UL)
#define BSP_BIT_22                      (0x00400000UL)
#define BSP_BIT_21                      (0x00200000UL)
#define BSP_BIT_20                      (0x00100000UL)
#define BSP_BIT_19                      (0x00080000UL)
#define BSP_BIT_18                      (0x00040000UL)
#define BSP_BIT_17                      (0x00020000UL)
#define BSP_BIT_16                      (0x00010000UL)
#define BSP_BIT_15                      (0x00008000UL)
#define BSP_BIT_14                      (0x00004000UL)
#define BSP_BIT_13                      (0x00002000UL)
#define BSP_BIT_12                      (0x00001000UL)
#define BSP_BIT_11                      (0x00000800UL)
#define BSP_BIT_10                      (0x00000400UL)
#define BSP_BIT_09                      (0x00000200UL)
#define BSP_BIT_08                      (0x00000100UL)
#define BSP_BIT_07                      (0x00000080UL)
#define BSP_BIT_06                      (0x00000040UL)
#define BSP_BIT_05                      (0x00000020UL)
#define BSP_BIT_04                      (0x00000010UL)
#define BSP_BIT_03                      (0x00000008UL)
#define BSP_BIT_02                      (0x00000004UL)
#define BSP_BIT_01                      (0x00000002UL)
#define BSP_BIT_00                      (0x00000001UL)
#define BSP_BIT_NONE                    (0x00000000UL)

#define BSP_BitSet(val, mask)           ((val) = ((val) | (mask)))
#define BSP_BitClr(val, mask)           ((val) = ((val) & ~(mask)))

#define BSP_BitSet32(val, mask)         (BSP_BitSet((val), (mask)))
#define BSP_BitClr32(val, mask)         (BSP_BitClr((val), (mask)))

#ifndef BSP_NOP_DELAY
#define BSP_NOP_DELAY()                 {__asm__ ("nop");}
#endif

#define SYS_PWR_EN                      (GPIO_GPC(2UL))

/* SNOR ROM Information Header */
#define SNOR_ROM_ID                     (0x524F4E53UL) // "SNOR"
#define SNOR_SECTION_MAX_COUNT          (30)

#define SNOR_SFMC_INIT_HEADER_ID        (0UL)
#define SNOR_MASTER_CERTI_ID            (1UL)
#define SNOR_HSM_BINARY_ID              (2UL)
#define SNOR_BL1_BINARY_ID              (3UL)
#define SNOR_MICOM_SUB_BINARY_ID        (4UL)
#define SNOR_UPDATE_FLAG_ID             (5UL)
#define SNOR_MICOM_BINARY_ID            (6UL)

typedef struct SNORSectionInfo
{
    uint32                              siOffset;
    uint32                              siSectionSize;
    uint32                              siDataSize;
    uint32                              siReserved;

} SNORSectionInfo_t;

typedef struct SNORRomInfo
{
    SNORSectionInfo_t                   riSectionInfo[SNOR_SECTION_MAX_COUNT];
    uint32                              riReserved[4];
    uint32                              riDebugEnable;
    uint32                              riDebugPort;
    uint32                              riRomId; //= SNOR_ROM_ID;
    uint32                              riCrc;

} SNORRomInfo_t; /* total 512byte */

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
*                                          BSP_UndefAbortExceptionHook
*
* This function is to hook the handle, before Handling exceptions
*
* @param    uiExceptId [in] ARM Exception ID
* @param    uiDummy [in] Dummy
* @param    uiLinkReg [in] Link Register(Return PC)
* @return
*
* Notes
*
***************************************************************************************************
*/

void BSP_UndefAbortExceptionHook
(
    uint32                              uiExceptId,
    uint32                              uiDummy,
    uint32                              uiLinkReg
);

/*
***************************************************************************************************
*                                          BSP_SystemHalt
*
* This function is designed for very restricted use case such as firmware upgrading.
*
* Notes
*
***************************************************************************************************
*/

void BSP_SystemHalt
(
    void
);

 /*
***************************************************************************************************
*                                          BSP_PreInit
*
* Enable the device driver for early starting the function such as clock and interrupt.
*
* @param    none.
* @return   void
*
* Notes
*
***************************************************************************************************
*/

void BSP_PreInit
(
    void
);

 /*
***************************************************************************************************
*                                          BSP_Init
*
* This function is to initialize BSP(board support package) base
*
* @param    none.
* @return   void
*
* Notes
*
***************************************************************************************************
*/

void BSP_Init
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

