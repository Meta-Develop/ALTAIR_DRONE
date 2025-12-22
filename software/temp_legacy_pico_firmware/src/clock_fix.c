#include <time.h>
#include "pico/stdlib.h"

int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    // Basic implementation for CLOCK_REALTIME / CLOCK_MONOTONIC
    uint64_t now_us = to_us_since_boot(get_absolute_time());
    tp->tv_sec = now_us / 1000000;
    tp->tv_nsec = (now_us % 1000000) * 1000;
    return 0;
}
