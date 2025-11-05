# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(t1s)

set(TICKLE_DIR ${CMAKE_CURRENT_LIST_DIR}/tickle)
set(TICKLE_INCLUDE_DIRS ${TICKLE_DIR}/include)
set(TICKLE_SOURCES
  ${TICKLE_DIR}/src/encoding.c
  ${TICKLE_DIR}/src/hal_zephyr.c
  ${TICKLE_DIR}/src/log.c
  ${TICKLE_DIR}/src/tickle.c
)

target_include_directories(app PRIVATE ${TICKLE_INCLUDE_DIRS})
target_sources(app PRIVATE src/main.c src/lan8651.c src/arp.c src/spi.c src/perf.c src/udp.c ${TICKLE_SOURCES} src/pubsub/ping.c src/pubsub/pong.c src/pubsub/type/Perf.c)

