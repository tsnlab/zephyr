/*
***************************************************************************************************
*
*   FileName : mpu.h
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

#ifndef MCU_BSP_MPU_HEADER
#define MCU_BSP_MPU_HEADER

#include <sal_api.h>

#define MPU_REGION_32B                  (0x00000004U)
#define MPU_REGION_64B                  (0x00000005U)
#define MPU_REGION_128B                 (0x00000006U)
#define MPU_REGION_256B                 (0x00000007U)
#define MPU_REGION_512B                 (0x00000008U)
#define MPU_REGION_1KB                  (0x00000009U)
#define MPU_REGION_2KB                  (0x0000000AU)
#define MPU_REGION_4KB                  (0x0000000BU)
#define MPU_REGION_8KB                  (0x0000000CU)
#define MPU_REGION_16KB                 (0x0000000DU)
#define MPU_REGION_32KB                 (0x0000000EU)
#define MPU_REGION_64KB                 (0x0000000FU)
#define MPU_REGION_128KB                (0x00000010U)
#define MPU_REGION_256KB                (0x00000011U)
#define MPU_REGION_512KB                (0x00000012U)
#define MPU_REGION_1MB                  (0x00000013U)
#define MPU_REGION_2MB                  (0x00000014U)
#define MPU_REGION_4MB                  (0x00000015U)
#define MPU_REGION_8MB                  (0x00000016U)
#define MPU_REGION_16MB                 (0x00000017U)
#define MPU_REGION_32MB                 (0x00000018U)
#define MPU_REGION_64MB                 (0x00000019U)
#define MPU_REGION_128MB                (0x0000001AU)
#define MPU_REGION_256MB                (0x0000001BU)
#define MPU_REGION_512MB                (0x0000001CU)
#define MPU_REGION_1GB                  (0x0000001DU)
#define MPU_REGION_2GB                  (0x0000001EU)
#define MPU_REGION_4GB                  (0x0000001FU)

#define MPU_REGION_EN                   (0x00000001U)

/* GPRs */
#define MPU_ARM_GPR_0                   (r0)
#define MPU_ARM_GPR_1                   (r1)
#define MPU_ARM_GPR_2                   (r2)
#define MPU_ARM_GPR_3                   (r3)
#define MPU_ARM_GPR_4                   (r4)
#define MPU_ARM_GPR_5                   (r5)
#define MPU_ARM_GPR_6                   (r6)
#define MPU_ARM_GPR_7                   (r7)
#define MPU_ARM_GPR_8                   (r8)
#define MPU_ARM_GPR_9                   (r9)
#define MPU_ARM_GPR_10                  (r10)
#define MPU_ARM_GPR_11                  (r11)
#define MPU_ARM_GPR_12                  (r12)
#define MPU_ARM_GPR_13                  (r13)
#define MPU_ARM_GPR_14                  (r14)
#define MPU_ARM_GPR_15                  (r15)
#define MPU_ARM_CPSR                    (cpsr)

/* Coprocessor number defines */
#define MPU_ARM_CP_0                    (0)
#define MPU_ARM_CP_1                    (1)
#define MPU_ARM_CP_2                    (2)
#define MPU_ARM_CP_3                    (3)
#define MPU_ARM_CP_4                    (4)
#define MPU_ARM_CP_5                    (5)
#define MPU_ARM_CP_6                    (6)
#define MPU_ARM_CP_7                    (7)
#define MPU_ARM_CP_8                    (8)
#define MPU_ARM_CP_9                    (9)
#define MPU_ARM_CP_10                   (10)
#define MPU_ARM_CP_11                   (11)
#define MPU_ARM_CP_12                   (12)
#define MPU_ARM_CP_13                   (13)
#define MPU_ARM_CP_14                   (14)
#define MPU_ARM_CP_15                   (15)

/* Coprocessor control register defines */
#define MPU_ARM_CR_0                    (cr0)
#define MPU_ARM_CR_1                    (cr1)
#define MPU_ARM_CR_2                    (cr2)
#define MPU_ARM_CR_3                    (cr3)
#define MPU_ARM_CR_4                    (cr4)
#define MPU_ARM_CR_5                    (cr5)
#define MPU_ARM_CR_6                    (cr6)
#define MPU_ARM_CR_7                    (cr7)
#define MPU_ARM_CR_8                    (cr8)
#define MPU_ARM_CR_9                    (cr9)
#define MPU_ARM_CR_10                   (cr10)
#define MPU_ARM_CR_11                   (cr11)
#define MPU_ARM_CR_12                   (cr12)
#define MPU_ARM_CR_13                   (cr13)
#define MPU_ARM_CR_14                   (cr14)
#define MPU_ARM_CR_15                   (cr15)

/* Current Processor Status Register (CPSR) Bits */
#define MPU_CPSR_THUMB_MODE             (0x20U)
#define MPU_CPSR_MODE_BITS              (0x1FU)
#define MPU_CPSR_SYSTEM_MODE            (0x1FU)
#define MPU_CPSR_UNDEFINED_MODE         (0x1BU)
#define MPU_CPSR_DATA_ABORT_MODE        (0x17U)
#define MPU_CPSR_SVC_MODE               (0x13U)
#define MPU_CPSR_IRQ_MODE               (0x12U)
#define MPU_CPSR_FIQ_MODE               (0x11U)
#define MPU_CPSR_USER_MODE              (0x10U)
#define MPU_CPSR_IRQ_ENABLE             (0x80U)
#define MPU_CPSR_FIQ_ENABLE             (0x40U)
#define MPU_CPSR_N_BIT                  (0x80000000U)
#define MPU_CPSR_Z_BIT                  (0x40000000U)
#define MPU_CPSR_C_BIT                  (0x20000000U)
#define MPU_CPSR_V_BIT                  (0x10000000U)

#define MPU_SHAREABLE                   (0x00000004U)           // shareable
#define MPU_STRONG_ORDERED_SHARED       (0x00000000U)           // strongly ordered, always shareable

#define MPU_DEVICE_SHARED               (0x00000001U)           // device, shareable
#define MPU_DEVICE_NONSHARED            (0x00000010U)           // device, non shareable

#define MPU_NORM_NSHARED_WT_NWA         (0x00000002U)           // Outer and Inner write-through, no write-allocate non-shareable
#define MPU_NORM_SHARED_WT_NWA          (0x00000006U)           // Outer and Inner write-through, no write-allocate shareable

#define MPU_NORM_NSHARED_WB_NWA         (0x00000003U)           // Outer and Inner write-back, no write-allocate non shareable
#define MPU_NORM_SHARED_WB_NWA          (0x00000007U)           // Outer and Inner write-back, no write-allocate shareable

#define MPU_NORM_NSHARED_NCACHE         (0x00000008U)           // Outer and Inner Non cacheable  non shareable
#define MPU_NORM_SHARED_NCACHE          (0x0000000CU)           // Outer and Inner Non cacheable shareable

#define MPU_NORM_NSHARED_WB_WA          (0x0000000BU)           // Outer and Inner write-back non shared
#define MPU_NORM_SHARED_WB_WA           (0x0000000FU)           // Outer and Inner write-back shared

/* inner and outer cache policies can be combined for different combinations */
#define MPU_NORM_IN_POLICY_NCACHE       (0x00000020U)           // inner non cacheable
#define MPU_NORM_IN_POLICY_WB_WA        (0x00000021U)           // inner write back write allocate
#define MPU_NORM_IN_POLICY_WT_NWA       (0x00000022U)           // inner write through no write allocate
#define MPU_NORM_IN_POLICY_WB_NWA       (0x00000023U)           // inner write back no write allocate

#define MPU_NORM_OUT_POLICY_NCACHE      (0x00000020U)           // outer non cacheable
#define MPU_NORM_OUT_POLICY_WB_WA       (0x00000028U)           // outer write back write allocate
#define MPU_NORM_OUT_POLICY_WT_NWA      (0x00000030U)           // outer write through no write allocate
#define MPU_NORM_OUT_POLICY_WB_NWA      (0x00000038U)           // outer write back no write allocate

#define MPU_NO_ACCESS                   ( 0x00000000U << 8U )   // No access
#define MPU_PRIV_RW_USER_NA             ( 0x00000001U << 8U )   // Privileged access only
#define MPU_PRIV_RW_USER_RO             ( 0x00000002U << 8U )   // Writes in User mode generate permission faults
#define MPU_PRIV_RW_USER_RW             ( 0x00000003U << 8U )   // Full Access
#define MPU_PRIV_RO_USER_NA             ( 0x00000005U << 8U )   // Privileged eead only
#define MPU_PRIV_RO_USER_RO             ( 0x00000006U << 8U )   // Privileged/User read-only

#define MPU_EXECUTE_NEVER               ( 0x00000001U << 12U )  // Bit 12

/* CP15 defines */
/* C0 Register defines */
#define MPU_ARM_CP15_MAIN_ID                "p15, 0, %0,  c0,  c0, 0"
#define MPU_ARM_CP15_CACHE_TYPE             "p15, 0, %0,  c0,  c0, 1"
#define MPU_ARM_CP15_TCM_TYPE               "p15, 0, %0,  c0,  c0, 2"
#define MPU_ARM_CP15_TLB_TYPE               "p15, 0, %0,  c0,  c0, 3"
#define MPU_ARM_CP15_MPU_TYPE               "p15, 0, %0,  c0,  c0, 4"
#define MPU_ARM_CP15_MULTI_PROC_AFFINITY    "p15, 0, %0,  c0,  c0, 5"
#define MPU_ARM_CP15_PROC_FEATURE_0         "p15, 0, %0,  c0,  c1, 0"
#define MPU_ARM_CP15_PROC_FEATURE_1         "p15, 0, %0,  c0,  c1, 1"
#define MPU_ARM_CP15_DEBUG_FEATURE_0        "p15, 0, %0,  c0,  c1, 2"
#define MPU_ARM_CP15_CACHE_SIZE_ID          "p15, 1, %0,  c0,  c0, 0"
#define MPU_ARM_CP15_CACHE_LEVEL_ID         "p15, 1, %0,  c0,  c0, 1"
#define MPU_ARM_CP15_AUXILARY_ID            "p15, 1, %0,  c0,  c0, 7"
#define MPU_ARM_CP15_CACHE_SIZE_SEL         "p15, 2, %0,  c0,  c0, 0"

/* C1 Register Defines */
#define MPU_ARM_CP15_SYS_CONTROL            "p15, 0, %0,  c1,  c0, 0"
#define MPU_ARM_CP15_AUX_CONTROL            "p15, 0, %0,  c1,  c0, 1"
#define MPU_ARM_CP15_CP_ACCESS_CONTROL      "p15, 0, %0,  c1,  c0, 2"

/* ARM_CP15_CONTROL bit defines */
#define MPU_ARM_CP15_CONTROL_TE         (0x40000000U)           //Thumb Execution
#define MPU_ARM_CP15_CONTROL_AFE        (0x20000000U)           //Access Flag Enable
#define MPU_ARM_CP15_CONTROL_TRE        (0x10000000U)           //TEX remap enable
#define MPU_ARM_CP15_CONTROL_NMFI       (0x08000000U)           //Nonmaskable Fast Interrupt Enable
#define MPU_ARM_CP15_CONTROL_EE         (0x02000000U)           //Exception endianess
#define MPU_ARM_CP15_CONTROL_RR         (0x00004000U)           //Round-robin bit
#define MPU_ARM_CP15_CONTROL_V          (0x00002000U)           //Base location of Exception register
#define MPU_ARM_CP15_CONTROL_I          (0x00001000U)           //Insturction Cache Enable
#define MPU_ARM_CP15_CONTROL_Z          (0x00000800U)           //Branch Prediction Enable
#define MPU_ARM_CP15_CONTROL_C          (0x00000004U)           //Enable Data Cache
#define MPU_ARM_CP15_CONTROL_A          (0x00000002U)           //Strict Alignment
#define MPU_ARM_CP15_CONTROL_M          (0x00000001U)           //MPU enable

/* C5 */
#define MPU_ARM_CP15_DATA_FAULT_STATUS      "p15, 0, %0,  c5,  c0, 0"
#define MPU_ARM_CP15_INST_FAULT_STATUS      "p15, 0, %0,  c5,  c0, 1"
#define MPU_ARM_CP15_AUX_DATA_FAULT_STATUS  "p15, 0, %0,  c5,  c1, 0"
#define MPU_ARM_CP15_AUX_INST_FAULT_STATUS  "p15, 0, %0,  c5,  c1, 1"

/* C6 */
#define MPU_ARM_CP15_DATA_FAULT_ADDRESS     "p15, 0, %0,  c6,  c0, 0"
#define MPU_ARM_CP15_INST_FAULT_ADDRESS     "p15, 0, %0,  c6,  c0, 2"
#define MPU_ARM_CP15_MPU_REG_BASEADDR       "p15, 0, %0,  c6,  c1, 0"
#define MPU_ARM_CP15_MPU_REG_SIZE_EN        "p15, 0, %0,  c6,  c1, 2"
#define MPU_ARM_CP15_MPU_REG_ACCESS_CTRL    "p15, 0, %0,  c6,  c1, 4"
#define MPU_ARM_CP15_MPU_MEMORY_REG_NUMBER  "p15, 0, %0,  c6,  c2, 0"

/* C7 */
#define MPU_ARM_CP15_NOP                    "p15, 0, %0,  c7,  c0, 4"
#define MPU_ARM_CP15_INVAL_IC_POU           "p15, 0, %0,  c7,  c5, 0"
#define MPU_ARM_CP15_INVAL_IC_LINE_MVA_POU  "p15, 0, %0,  c7,  c5, 1"
#define MPU_ARM_CP15_INVAL_BTAC             "p15, 0, %0,  c7,  c5, 6"

/* C9 */
#define MPU_ARM_CP15_ATCM_REG_SIZE_ADDR     "p15, 0, %0,  c9, c1, 1"
#define MPU_ARM_CP15_BTCM_REG_SIZE_ADDR     "p15, 0, %0,  c9, c1, 0"
#define MPU_ARM_CP15_TCM_SELECTION          "p15, 0, %0,  c9, c2, 0"
#define MPU_ARM_CP15_PERF_MONITOR_CTRL      "p15, 0, %0,  c9, c12, 0"
#define MPU_ARM_CP15_COUNT_ENABLE_SET       "p15, 0, %0,  c9, c12, 1"
#define MPU_ARM_CP15_COUNT_ENABLE_CLR       "p15, 0, %0,  c9, c12, 2"
#define MPU_ARM_CP15_V_FLAG_STATUS          "p15, 0, %0,  c9, c12, 3"
#define MPU_ARM_CP15_SW_INC                 "p15, 0, %0,  c9, c12, 4"
#define MPU_ARM_CP15_EVENT_CNTR_SEL         "p15, 0, %0,  c9, c12, 5"
#define MPU_ARM_CP15_PERF_CYCLE_COUNTER     "p15, 0, %0,  c9, c13, 0"
#define MPU_ARM_CP15_EVENT_TYPE_SEL         "p15, 0, %0,  c9, c13, 1"
#define MPU_ARM_CP15_PERF_MONITOR_COUNT     "p15, 0, %0,  c9, c13, 2"
#define MPU_ARM_CP15_USER_ENABLE            "p15, 0, %0,  c9, c14, 0"
#define MPU_ARM_CP15_INTR_ENABLE_SET        "p15, 0, %0,  c9, c14, 1"
#define MPU_ARM_CP15_INTR_ENABLE_CLR        "p15, 0, %0,  c9, c14, 2"

/* C15 */
#define MPU_ARM_CP15_SEC_AUX_CTRL           "p15, 0, %0, c15,  c0, 0"

/* CP15 operations */
//Write Control Register configuration data
#define MPU_WriteControlRegisterConfigurationData(rn, v)    __asm__ __volatile__("mcr " rn : : "r" (v));


//Read Control Register configuration data
#define MPU_ReadControlRegisterConfigurationData(rn, v)    __asm__ __volatile__("mrc " rn : "=r" (v));

/* DMA */
extern uint32 __nc_dmastart;
extern uint32 _end_of_nc_dma;

/* CAN */
extern uint32 __nc_canstart;
extern uint32 _end_of_nc_can;




// MPU DEFAULT DEFINES
#define MPU_MAX_REGION                  (15U)                   //SYSTEEM DEFINED // 16U(MAX)
#define MPU_CAN_INDEX                   (1u)
#define MPU_DMA_INDEX                   (2u)

// MPU DEFINES
#define MPU_REGION_ENABLE               (1U)
#define MPU_REGION_DISABLE              (0U)

// MPU START ADDR

#define MPU_PERI_ADDR                   (0xA0000000U)
#define MPU_PFLASHCON_ADDR              (0xA1000000U)
#define MPU_PFLASH_OPTAREA_ADDR         (0xA1010000U)
#define MPU_PFLASH_INT_ADDR             (0xA1020000U)

#define MPU_DFLASHCON_ADDR              (0xA1080000U)
#define MPU_DFLASH_OPTAREA_ADDR         (0xA1090000U)
#define MPU_DFLASH_INT_ADDR             (0xA10A0000U)

#define MPU_SFMC_ADDR                   (0xA0F00000U)

#define MPU_SRAM0_ADDR                  (0x00000000U)
#define MPU_REMAP_ADDR                  (0x01000000U)
#define MPU_PFLASH_ADDR                 (0x20000000U)
#define MPU_DFLASH_ADDR                 (0x30000000U)
#define MPU_SNOR_ADDR                   (0x40000000U)



typedef struct MPUConfig
{
    uint32                              uiRegionEnable;
    uint32                              uiRegionBase;
    uint32                              uiRegionSize;
    uint32                              uiRegionAttr;
} MPUConfig_t;

/*
***************************************************************************************************
*                                       MPU_Init
*
* Initialize MPU.
*
* @param    none
* @return   void
*
* Notes
*
***************************************************************************************************
*/

void MPU_Init
(
    void
);

/*
***************************************************************************************************
*                                       MPU_GetDMABaseAddress
*
* Registers  functions needed by specific application.
*
* @param    none
* @return   uint32 DMA Base Address.
*
* Notes
*
***************************************************************************************************
*/

uint32 MPU_GetDMABaseAddress
(
    void
);

/*
***************************************************************************************************
*                                       MPU_GetCANMessageRAMAddress
*
* Registers  functions needed by specific application.
*
* @param    none
* @return   uint32 CAN Message RAM Base Address.
*
* Notes
*
***************************************************************************************************
*/

uint32 MPU_GetCANBaseAddress
(
    void
);

#endif  // MCU_BSP_MPU_HEADER

