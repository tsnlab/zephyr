/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>

#if 0
   31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 |   |   |   | XN|   |     AP    |   |   |    TEX    | S | C | B |
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
#endif

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

#define MPU_NO_ACCESS                   ( 0x00000000U << 8U )   // No access
#define MPU_PRIV_RW_USER_NA             ( 0x00000001U << 8U )   // Privileged access only
#define MPU_PRIV_RW_USER_RO             ( 0x00000002U << 8U )   // Writes in User mode generate permission faults
#define MPU_PRIV_RW_USER_RW             ( 0x00000003U << 8U )   // Full Access
#define MPU_PRIV_RO_USER_NA             ( 0x00000005U << 8U )   // Privileged eead only
#define MPU_PRIV_RO_USER_RO             ( 0x00000006U << 8U )   // Privileged/User read-only

extern uint32_t _image_rom_end_order;
static const struct arm_mpu_region mpu_regions[] = {
	MPU_REGION_ENTRY("SRAM0",
			0x00000000,
			REGION_512K,
			{  .rasr = (MPU_NORM_SHARED_WB_WA | MPU_PRIV_RW_USER_RW) }),

	MPU_REGION_ENTRY("SRAM00",
			0x00000000,
			0,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("SRAM01",
			0x00000000,
			0,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("REMAP",
			0x01000000,
			REGION_4M,
			{ .rasr = MPU_NORM_NSHARED_WB_NWA | MPU_PRIV_RO_USER_RO}),

	MPU_REGION_ENTRY("PFLASH",
			0x20000000,
			REGION_2M,
			{ .rasr = MPU_NORM_NSHARED_WB_NWA | MPU_PRIV_RO_USER_RO}),

	MPU_REGION_ENTRY("DFLASH",
			0x30000000,
			REGION_256K,
			{ .rasr = MPU_NORM_NSHARED_WB_NWA | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("SNOR",
			0x40000000,
			REGION_256M,
			{ .rasr = MPU_NORM_NSHARED_NCACHE | MPU_PRIV_RO_USER_RO}),

	MPU_REGION_ENTRY("PERI",
			0xA0000000,
			REGION_16M,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("PFLASHCON",
			0xA1000000,
			REGION_64K,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("PFLASH_OPTAREA",
			0xA1010000,
			REGION_16K,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RO}),

	MPU_REGION_ENTRY("PFLASH_INT",
			0xA1020000,
			REGION_32B,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("DFLASHCON",
			0xA1080000,
			REGION_64K,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),

	MPU_REGION_ENTRY("DFLASH_OPTAREA",
			0xA1090000,
			REGION_2K,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RO}),

	MPU_REGION_ENTRY("DFLASH_INT",
			0xA10A0000,
			REGION_64B,
			{ .rasr = MPU_STRONG_ORDERED_SHARED | MPU_PRIV_RW_USER_RW}),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};

