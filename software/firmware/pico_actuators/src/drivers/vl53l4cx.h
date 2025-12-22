#ifndef VL53L4CX_H
#define VL53L4CX_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define VL53L4CX_ADDR 0x29

typedef struct {
    uint16_t range_mm;
} vl53_data_t;

bool vl53_init(i2c_inst_t *i2c);
void vl53_read_data(i2c_inst_t *i2c, vl53_data_t *data);

#endif
