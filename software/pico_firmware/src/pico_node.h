#ifndef PICO_NODE_H
#define PICO_NODE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "drivers/bno055.h"

// Shared Data
typedef struct {
    float motors[6];
    TickType_t last_update_time;
} motor_state_t;

typedef struct {
    bno055_data_t imu;
    float esc_rpm[6];
} sensor_state_t;

extern motor_state_t g_motor_state;
extern SemaphoreHandle_t g_motor_mutex;

extern sensor_state_t g_sensor_state;
extern SemaphoreHandle_t g_sensor_mutex;

void pico_node_init(void);

// Task Functions
void task_control(void *params);
void task_microros(void *params);

#endif
