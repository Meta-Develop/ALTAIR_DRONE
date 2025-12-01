#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BNO055_ADDR 0x28 // Default I2C address

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} bno055_data_t;

void bno055_init(i2c_inst_t *i2c_instance, uint sda, uint scl);
bool bno055_read_raw(bno055_data_t *data);

#endif
