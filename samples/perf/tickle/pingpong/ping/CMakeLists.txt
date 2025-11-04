# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(perf_tickle_ping)

set(TICKLE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../tickle)
set(TICKLE_INCLUDE_DIRS ${TICKLE_DIR}/include)
set(TICKLE_SOURCES
  ${TICKLE_DIR}/src/encoding.c
  ${TICKLE_DIR}/src/hal_zephyr.c
  ${TICKLE_DIR}/src/log.c
  ${TICKLE_DIR}/src/tickle.c
)

set(LAN8651_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../lan8651)
set(LAN8651_INCLUDE_DIRS ${LAN8651_DIR}/include)
set(LAN8651_SOURCES
  ${LAN8651_DIR}/lan8651.c
  ${LAN8651_DIR}/spi.c
)

target_include_directories(app PRIVATE ${TICKLE_INCLUDE_DIRS} ${LAN865X_INCLUDE_DIRS})
target_sources(app PRIVATE src/main.c src/Command.c src/State.c ${TICKLE_SOURCES} ${LAN865X_SOURCES})
