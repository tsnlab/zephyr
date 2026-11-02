cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_dtoe)
# the same application as DTOE examples/hello_dtoe, built with Zephyr on core 0 and libdtoe.a on core 1
target_sources(app PRIVATE src/main.c)
