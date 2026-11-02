cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(data_transfer_test)
# DTOE test firmware test/functional/data_transfer_test, its main.c unchanged, with Zephyr on core 0
include(../common/dtoe_source.cmake)
target_sources(app PRIVATE ${DTOE_SOURCE_DIR}/test/functional/data_transfer_test/main.c ../common/bootsel.c)
target_include_directories(app PRIVATE ../common/pico_compat)
