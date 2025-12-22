#ifndef VL53L4CX_H
#define VL53L4CX_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Default I2C address (may be shifted left by some libs)
#define VL53L4CX_ADDR 0x29

// Key Registers (simplified from ST datasheet)
#define VL53L4CX_IDENTIFICATION_MODEL_ID   0x010F
#define VL53L4CX_FIRMWARE_SYSTEM_STATUS    0x00E5
#define VL53L4CX_SYSTEM_START              0x0087
#define VL53L4CX_RESULT_RANGE_STATUS       0x0089
#define VL53L4CX_RESULT_FINAL_RANGE_MM     0x0096  // 16-bit
#define VL53L4CX_SYSTEM_INTERRUPT_CLEAR    0x0086

// Expected Model ID
#define VL53L4CX_MODEL_ID_VAL  0xEB  // VL53L4CX

typedef struct {
    uint16_t range_mm;
    uint8_t  status;
} vl53_data_t;

// Initialize VL53L4CX sensor
bool vl53_init(i2c_inst_t *i2c);

// Start a single ranging measurement
void vl53_start_ranging(i2c_inst_t *i2c);

// Check if measurement is complete
bool vl53_data_ready(i2c_inst_t *i2c);

// Read distance in mm
void vl53_read_data(i2c_inst_t *i2c, vl53_data_t *data);

// Stop ranging
void vl53_stop_ranging(i2c_inst_t *i2c);

#endif
