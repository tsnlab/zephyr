cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(async_stream_test)
# DTOE test firmware test/performance/async_stream_test, its main.c unchanged, with Zephyr on core 0
include(../common/dtoe_source.cmake)
target_sources(app PRIVATE
    ${DTOE_SOURCE_DIR}/test/performance/async_stream_test/main.c
    ../common/bootsel.c
)
target_include_directories(app PRIVATE ../common/pico_compat ${DTOE_SOURCE_DIR}/typesupport)
# state period and setpoint reliability are fixed at build time, as in the pico-sdk images
set(STATE_PERIOD_US 2000 CACHE STRING "state period in microseconds: 10000, 2000, 1000 or 500")
option(SP_RELIABLE "setpoint reader RELIABLE" ON)
target_compile_definitions(app PRIVATE STATE_PERIOD_US=${STATE_PERIOD_US} $<$<BOOL:${SP_RELIABLE}>:SP_RELIABLE=1>)
