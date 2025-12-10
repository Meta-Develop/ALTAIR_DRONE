#ifndef PICO_NODE_H
#define PICO_NODE_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "drivers/mpu6050.h"

// Shared Data (Queue uses this type)
typedef struct {
    uint64_t timestamp_us;
    mpu6050_data_t imu;
    float esc_rpm[6]; // Added
} sensor_packet_t;

// Shared Data (State)
typedef struct {
    volatile float motors[6];
    volatile uint32_t last_update_time_ms;
} motor_state_t;

extern motor_state_t g_motor_state;

// Main Initialization
void pico_node_init(void);

#endif
