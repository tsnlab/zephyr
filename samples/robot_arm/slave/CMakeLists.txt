# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot_arm_slave)

add_subdirectory(tickle)  # TODO: Include tickle as external module

target_include_directories(app PRIVATE ${TICKLE_INCLUDE_DIRS})
target_sources(app PRIVATE src/main.c src/Command.c src/State.c ${TICKLE_SOURCES})
