# DTOE sources, for the applications that compile a DTOE test source unchanged (test/<group>/<test>/main.c).
# The DTOE library itself always comes from the binary distribution (ZEPHYR_EXTRA_MODULES).
# Set with -DDTOE_SOURCE_DIR=<DTOE repository> or the environment variable DTOE_SOURCE_DIR.
if(NOT DTOE_SOURCE_DIR AND DEFINED ENV{DTOE_SOURCE_DIR})
  set(DTOE_SOURCE_DIR $ENV{DTOE_SOURCE_DIR})
endif()
set(DTOE_SOURCE_DIR "${DTOE_SOURCE_DIR}" CACHE PATH "DTOE repository (test sources)")
if(NOT EXISTS "${DTOE_SOURCE_DIR}/test")
  message(FATAL_ERROR "DTOE_SOURCE_DIR (${DTOE_SOURCE_DIR}) is not a DTOE repository: "
                      "set -DDTOE_SOURCE_DIR=<DTOE repository> or the environment variable DTOE_SOURCE_DIR")
endif()
