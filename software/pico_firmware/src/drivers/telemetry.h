#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

// Telemetry Pins (Assumed 8-13)
#define TELEMETRY_PIN_START 8
#define NUM_ESC 6

void telemetry_init(PIO pio, uint sm);
float telemetry_read_rpm(PIO pio, uint sm, uint motor_index);

#endif
