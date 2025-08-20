# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(t1s)

target_sources(app PRIVATE src/main.c src/lan8651.c src/arp.c src/spi.c)

