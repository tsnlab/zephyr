cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flash_write_test)
# DTOE async streaming test firmware (test/performance/async_stream_test/main.c, unchanged) with a Zephyr thread
# that erases and writes the flash meanwhile: core 1 must keep running while core 0 has the flash out of XIP.
set(DTOE_SOURCE_DIR $ENV{HOME}/workspaces/TSNLab-PICO-C/libraries/2026-wiznet-dtoe CACHE PATH "DTOE sources (test firmware)")
set(STATE_PERIOD_US 2000 CACHE STRING "state period of the async streaming test")
target_sources(app PRIVATE
    ${DTOE_SOURCE_DIR}/test/performance/async_stream_test/main.c
    src/flash_writer.c
    ../common/bootsel.c
)
target_include_directories(app PRIVATE ../common/pico_compat)
target_compile_definitions(app PRIVATE STATE_PERIOD_US=${STATE_PERIOD_US} SP_RELIABLE=1)
