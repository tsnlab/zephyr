cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rtt_test)
# DTOE test firmware test/performance/rtt_test with Zephyr on core 0. Not the
# _cpuload variant: it needs the agent built with DTOE_CPU_LOAD, which the distribution is not.
target_sources(app PRIVATE src/main.c ../common/typesupport/msg_pingpong.c ../common/bootsel.c)
target_include_directories(app PRIVATE ../common/pico_compat ../common/typesupport)
