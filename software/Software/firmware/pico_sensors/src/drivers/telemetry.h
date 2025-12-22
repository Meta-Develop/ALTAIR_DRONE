#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

// 6 Separate Telemetry Pins (Discontiguous)
extern const uint telemetry_pins[NUM_ESC];

void telemetry_init(void);
float telemetry_read_rpm(uint motor_index);

#endif
