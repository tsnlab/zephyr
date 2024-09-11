/*
***************************************************************************************************
*
*   FileName : pmio_dev.h
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

#ifndef  MCU_BSP_PMIO_DEV_HEADER
#define  MCU_BSP_PMIO_DEV_HEADER

#if 1 // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <sal_internal.h>
#include "reg_phys.h"

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
/*PMIO VALUES*/
/*PMIO va: Bit flags*/
#define PMIO_VA_BITALL                  (0xFFFFFFFFUL)
#define PMIO_VA_BITCLEAR                (0x00000000UL)
#define PMIO_VA_BIT31                   (0x80000000UL)
#define PMIO_VA_BIT30                   (0x40000000UL)
#define PMIO_VA_BIT29                   (0x20000000UL)
#define PMIO_VA_BIT28                   (0x10000000UL)
#define PMIO_VA_BIT27                   (0x08000000UL)
#define PMIO_VA_BIT26                   (0x04000000UL)
#define PMIO_VA_BIT25                   (0x02000000UL)
#define PMIO_VA_BIT24                   (0x01000000UL)
#define PMIO_VA_BIT23                   (0x00800000UL)
#define PMIO_VA_BIT22                   (0x00400000UL)
#define PMIO_VA_BIT21                   (0x00200000UL)
#define PMIO_VA_BIT20                   (0x00100000UL)
#define PMIO_VA_BIT19                   (0x00080000UL)
#define PMIO_VA_BIT18                   (0x00040000UL)
#define PMIO_VA_BIT17                   (0x00020000UL)
#define PMIO_VA_BIT16                   (0x00010000UL)
#define PMIO_VA_BIT15                   (0x00008000UL)
#define PMIO_VA_BIT14                   (0x00004000UL)
#define PMIO_VA_BIT13                   (0x00002000UL)
#define PMIO_VA_BIT12                   (0x00001000UL)
#define PMIO_VA_BIT11                   (0x00000800UL)
#define PMIO_VA_BIT10                   (0x00000400UL)
#define PMIO_VA_BIT9                    (0x00000200UL)
#define PMIO_VA_BIT8                    (0x00000100UL)
#define PMIO_VA_BIT7                    (0x00000080UL)
#define PMIO_VA_BIT6                    (0x00000040UL)
#define PMIO_VA_BIT5                    (0x00000020UL)
#define PMIO_VA_BIT4                    (0x00000010UL)
#define PMIO_VA_BIT3                    (0x00000008UL)
#define PMIO_VA_BIT2                    (0x00000004UL)
#define PMIO_VA_BIT1                    (0x00000002UL)
#define PMIO_VA_BIT0                    (0x00000001UL)

/*PMIO va: Size*/
#define PMIO_VA_BACKUP_RAM_SIZE         (0UL) //Not support in this target

/*PMIO va: Port*/
#define PMIO_VA_LEAKAGE_PU_PORT         (PMIO_VA_BIT0)
#define PMIO_VA_LEAKAGE_PD_PORT         ( \
                                        PMIO_VA_BIT2 | \
                                        PMIO_VA_BIT3 | \
                                        PMIO_VA_BIT6 | \
                                        PMIO_VA_BIT9 | \
                                        PMIO_VA_BIT10 | \
                                        PMIO_VA_BIT11 | \
                                        PMIO_VA_BIT12 | \
                                        PMIO_VA_BIT15)
#define PMIO_VA_LEAKAGE_OUT_H_PORT      (PMIO_VA_BIT13)
#define PMIO_VA_LEAKAGE_OUT_L_PORT      (PMIO_VA_BIT8|PMIO_VA_BIT14)
#define PMIO_VA_LEAKAGE_IN_PORT         (PMIO_VA_BIT0 | PMIO_VA_BIT1 | \
                                        PMIO_VA_BIT2 | PMIO_VA_BIT3 | \
                                        PMIO_VA_BIT4 | PMIO_VA_BIT5 | \
                                        PMIO_VA_BIT6 | PMIO_VA_BIT7 | \
                                        PMIO_VA_BIT9 | PMIO_VA_BIT10 | \
                                        PMIO_VA_BIT11 | PMIO_VA_BIT12 | \
                                        PMIO_VA_BIT15)




/*PMIO va: Address*/
#define PMIO_VA_BASE_ADDR               (MCU_BSP_PMIO_BASE)        // R/W, Backup RAM Register
#define PMIO_VA_BACKUP_RAM_ADDR         (0x0UL)             // R/W, Backup RAM Register //Not support in this target
#define PMIO_VA_PMGPIO_CTL_ADDR         (0xA0F28828UL) //PMGPIO Controll Register
#define PMIO_VA_RST_STS0_ADDR           (0xC003F400UL) //Reset Status 0 Register (0x14400130)
#define PMIO_VA_RST_STS1_ADDR           (0xC003F404UL) //Reset Status 1 Register (0x14400134)
#define PMIO_VA_RST_STS2_ADDR           (0xC003F408UL) //Reset Status 2 Register (0x14400138)
//STR Defines
#define PMIO_VA_PMU_MICOM_USER_ADDR     (0x1B936178UL) //Save boot reason
#define PMIO_VA_PMU_WAKEUP_EN_ADDR      (0x1B936024UL) //Check wakeup by RTC
//#define RTCSTR                        0x16480000 + 0x4C //Check RTC boot reason

//efuse data
#define PMIO_VA_PMU_ECID_USER0_FBOUT0_ADDR  (0xA0F2804CUL)
#define PMIO_VA_PMU_ECID_USER0_FBOUT1_ADDR  (0xA0F28050UL)

//System configuration's GPIO Software reset
#define PMIO_VA_SYS_CONF_SW_RESET2_ADDR     (0xA0F20014UL)

/*PMIO REGISTERS*/
/*PMIO reg: GPK by PMIO*/
#define PMIO_REG_GPK_FDAT               (*(SALReg32 *)(0xA0F28A00UL)) //GPK Filtered Data Port
#define PMIO_REG_GPK_IRQST              (*(SALReg32 *)(0xA0F28A10UL)) //GPK Interrupt Status Register
#define PMIO_REG_GPK_IRQEN              (*(SALReg32 *)(0xA0F28A14UL)) //GPK Interrupt Enable Register
#define PMIO_REG_GPK_IRQPOL             (*(SALReg32 *)(0xA0F28A18UL)) //GPK Interrupt Polarity Register
#define PMIO_REG_GPK_IRQTM0             (*(SALReg32 *)(0xA0F28A1CUL)) //GPK Interrupt Trigger Mode 0 Reg
#define PMIO_REG_GPK_IRQTM1             (*(SALReg32 *)(0xA0F28A20UL)) //GPK Interrupt Trigger Mode 1 Reg
#define PMIO_REG_GPK_FCK                (*(SALReg32 *)(0xA0F28A24UL)) //GPK Filter Clock Conf Reg
#define PMIO_REG_GPK_FBP                (*(SALReg32 *)(0xA0F28A28UL)) //GPK Filter Bypass Register
#define PMIO_REG_GPK_CTL                (*(SALReg32 *)(0xA0F28A2CUL)) //GPK Miscellaneous Control Reg

/*PMIO reg: PMGPIO*/
#define PMIO_REG_PMGPIO_DAT             (*(SALReg32 *)(0xA0F28800UL)) //PMGPIO Data Reg
#define PMIO_REG_PMGPIO_EN              (*(SALReg32 *)(0xA0F28804UL)) //PMGPIO Direction Control Reg
#define PMIO_REG_PMGPIO_FS              (*(SALReg32 *)(0xA0F28808UL)) //PMGPIO Function Select Reg
#define PMIO_REG_PMGPIO_IEN             (*(SALReg32 *)(0xA0F2880CUL)) //PMGPIO Input Enable Reg
#define PMIO_REG_PMGPIO_PE              (*(SALReg32 *)(0xA0F28810UL)) //PMGPIO PULL Down/Up Enable Reg
#define PMIO_REG_PMGPIO_PS              (*(SALReg32 *)(0xA0F28814UL)) //PMGPIO PULL Down/Up Select Reg
#define PMIO_REG_PMGPIO_CD0             (*(SALReg32 *)(0xA0F28818UL)) //PMGPIO Driver Strength 1 Reg
#define PMIO_REG_PMGPIO_CD1             (*(SALReg32 *)(0xA0F2881CUL)) //PMGPIO Driver Strength 1 Reg
#define PMIO_REG_PMGPIO_EE0             (*(SALReg32 *)(0xA0F28820UL)) //PMGPIO Event Enable 0 Reg
#define PMIO_REG_PMGPIO_EE1             (*(SALReg32 *)(0xA0F28824UL)) //PMGPIO Event Enable 1 Reg
#define PMIO_REG_PMGPIO_CTL             (*(SALReg32 *)(0xA0F28828UL)) //PMGPIO Control Reg
#define PMIO_REG_PMGPIO_DI              (*(SALReg32 *)(0xA0F2882CUL)) //PMGPIO Input Raw Status Reg
#define PMIO_REG_PMGPIO_STR             (*(SALReg32 *)(0xA0F28830UL)) //PMGPIO Rising Event Status Reg
#define PMIO_REG_PMGPIO_STF             (*(SALReg32 *)(0xA0F28834UL)) //PMGPIO Falling Event Status Reg
#define PMIO_REG_PMGPIO_POL             (*(SALReg32 *)(0xA0F28838UL)) //PMGPIO Event Polarity Reg
#define PMIO_REG_PMGPIO_PROT            (*(SALReg32 *)(0xA0F2883CUL)) //PMGPIO Protect Reg
#define PMIO_REG_SW_TRIM0               (*(SALReg32 *)(0xA0F28840UL))
#define PMIO_REG_SW_TRIM1               (*(SALReg32 *)(0xA0F28844UL))
#define PMIO_REG_SW_LDO10               (*(SALReg32 *)(0xA0F28848UL))
#define PMIO_REG_TECT                   (*(SALReg32 *)(0xA0F2884CUL))
#define PMIO_REG_LVSE_EN                (*(SALReg32 *)(0xA0F28850UL))
#define PMIO_REG_PMGPIO_APB             (*(SALReg32 *)(0xA0F28A00UL))

/*
***************************************************************************************************
*                                             LOCAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

#endif  // ( MCU_BSP_SUPPORT_DRIVER_PMIO == 1 )

#endif  // MCU_BSP_PMIO_DEV_HEADER

