# Copyright (c) 2025 Junho Lee <junho@tsnlab.com>
# SPDX-License-Identifier: Apache-2.0

# TODO: This should be in "modules" directory but somehow it refuses to work
# TODO: It woule be best to include tickle as an external module

#if(CONFIG_TICKLE)

#set(TICKLE_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
set(TICKLE_DIR ${CMAKE_CURRENT_LIST_DIR})

#zephyr_library()

# zephyr_library_include_directories(
#   ${TICKLE_DIR}/include
# )
set(TICKLE_INCLUDE_DIRS ${TICKLE_DIR}/include PARENT_SCOPE)

# zephyr_library_sources(
#   ${TICKLE_DIR}/src/encoding.c
#   ${TICKLE_DIR}/src/hal_zephyr.c
#   ${TICKLE_DIR}/src/log.c
#   ${TICKLE_DIR}/src/tickle.c
# )
set(TICKLE_SOURCES
  ${TICKLE_DIR}/src/encoding.c
  ${TICKLE_DIR}/src/hal_zephyr.c
  ${TICKLE_DIR}/src/log.c
  ${TICKLE_DIR}/src/tickle.c
  PARENT_SCOPE
)

#endif()
