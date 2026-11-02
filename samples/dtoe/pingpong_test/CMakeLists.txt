cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pingpong_test)
# DTOE test firmware test/performance/pingpong_test, its main.c unchanged, with Zephyr on core 0
include(../common/dtoe_source.cmake)
target_sources(app PRIVATE
    ${DTOE_SOURCE_DIR}/test/performance/pingpong_test/main.c
    ${DTOE_SOURCE_DIR}/typesupport/msg_pingpong.c
    ../common/bootsel.c
)
target_include_directories(app PRIVATE ../common/pico_compat ${DTOE_SOURCE_DIR}/typesupport)

