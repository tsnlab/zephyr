cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(history_test)
# DTOE test firmware test/functional/history_test with Zephyr on core 0
# pingpong type support is application code, as in the pico-sdk build
target_sources(app PRIVATE src/main.c ../common/typesupport/msg_pingpong.c ../common/bootsel.c)
target_include_directories(app PRIVATE ../common/pico_compat ../common/typesupport)
