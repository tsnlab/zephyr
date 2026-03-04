# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Add SoC source files
zephyr_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)

# Make this directory available for includes
zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
