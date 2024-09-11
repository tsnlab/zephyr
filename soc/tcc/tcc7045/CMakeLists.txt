# Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(
  startup.S
  vector.S
  vfp_init.S
  arm_a.S
  sal.S
  #rtos.S
  # portASM.S
  soc.c
)
zephyr_sources_ifdef(
  CONFIG_ARM_MPU
  arm_mpu_regions.c
  mpu.c
  bsp.c
  port.c
  pmio_dev.c
)

zephyr_include_directories(.)

if(CONFIG_SOC_TCC7045)
  #set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld CACHE INTERNAL "")
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
