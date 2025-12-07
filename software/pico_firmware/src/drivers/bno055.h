#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BNO055_ADDR 0x28 // Default I2C address

// Registers
// Registers
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_ACC_CONFIG_ADDR 0x08
#define BNO055_GYR_CONFIG_0_ADDR 0x0A
#define BNO055_OPR_MODE_ADDR 0x3D

// Modes
#define OPMODE_CONFIG 0x00
#define OPMODE_AMG 0x07

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} bno055_data_t;

bool bno055_init(i2c_inst_t *i2c_instance, uint sda, uint scl);
int bno055_write_register(uint8_t reg, uint8_t value);
bool bno055_set_high_speed_mode(void);
bool bno055_read_raw(bno055_data_t *data);

#endif
