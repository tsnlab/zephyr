# Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(
	startup.S
	soc.c
)

zephyr_sources_ifdef(
  CONFIG_ARM_MPU
  arm_mpu_regions.c
)

zephyr_include_directories(.)

# Force using BFD-LD
zephyr_ld_options(-fuse-ld=bfd)

# Set compile options
zephyr_linker_sources(ROM_START SORT_KEY 0x0 init.ld)

if(CONFIG_SOC_TCC7045)
  #set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld CACHE INTERNAL "")
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
