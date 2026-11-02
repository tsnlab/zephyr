#ifndef DTOE_APPS_PICO_TIME_H
#define DTOE_APPS_PICO_TIME_H

#include <zephyr/kernel.h>

#include "pico/types.h"

static inline absolute_time_t get_absolute_time(void) {
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

static inline absolute_time_t make_timeout_time_us(uint64_t us) {
    return get_absolute_time() + us;
}

static inline absolute_time_t delayed_by_us(absolute_time_t t, uint64_t us) {
    return t + us;
}

static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to - from);
}

static inline void sleep_until(absolute_time_t t) {
    absolute_time_t now = get_absolute_time();

    if (t > now) {
        k_usleep((int32_t)(t - now));
    }
}

static inline void sleep_us(uint64_t us) {
    k_usleep((int32_t)us);
}

static inline void sleep_ms(uint32_t ms) {
    k_msleep((int32_t)ms);
}

static inline uint64_t time_us_64(void) {
    return get_absolute_time();
}

static inline absolute_time_t from_us_since_boot(uint64_t us) {
    return us;
}

// spins like pico-sdk; a thread of higher priority still preempts it
static inline void busy_wait_until(absolute_time_t t) {
    while (get_absolute_time() < t) {
    }
}

#endif /* DTOE_APPS_PICO_TIME_H */
