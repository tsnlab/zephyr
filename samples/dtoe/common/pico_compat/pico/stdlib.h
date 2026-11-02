#ifndef DTOE_APPS_PICO_STDLIB_H
#define DTOE_APPS_PICO_STDLIB_H

#include "pico/time.h"

// the application runs on core 0; core 1 is the DTOE agent
static inline unsigned int get_core_num(void) {
    return 0;
}

// console output is not buffered (printk, CDC ACM console)
static inline void stdio_flush(void) {
}

static inline void tight_loop_contents(void) {
}

#endif /* DTOE_APPS_PICO_STDLIB_H */
