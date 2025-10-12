# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ar4mk3_with_t1s)

target_sources(app PRIVATE main.c ../lib/lan8651.c ../lib/spi.c ../lib/control_msg.c)