cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(history_test)
# DTOE test firmware test/functional/history_test, its main.c unchanged, with Zephyr on core 0
# pingpong type support is application code, as in the pico-sdk build
set(DTOE_SOURCE_DIR $ENV{HOME}/workspaces/TSNLab-PICO-C/libraries/2026-wiznet-dtoe CACHE PATH "DTOE sources (test firmware)")
target_sources(app PRIVATE
    ${DTOE_SOURCE_DIR}/test/functional/history_test/main.c
    ${DTOE_SOURCE_DIR}/typesupport/msg_pingpong.c
    ../common/bootsel.c
)
target_include_directories(app PRIVATE ../common/pico_compat ${DTOE_SOURCE_DIR}/typesupport)
