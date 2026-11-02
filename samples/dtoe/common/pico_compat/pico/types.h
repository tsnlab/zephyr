/*
 * pico-sdk names used by the DTOE test firmware sources (test/<group>/<test>/main.c), on Zephyr: the Zephyr
 * applications compile those sources unchanged. Only what they use.
 */
#ifndef DTOE_APPS_PICO_TYPES_H
#define DTOE_APPS_PICO_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// microseconds since boot, compared with < as in pico-sdk release builds
typedef uint64_t absolute_time_t;

#endif /* DTOE_APPS_PICO_TYPES_H */
