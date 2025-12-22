#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP388_ADDR 0x77 // Or 0x76

typedef struct {
    float pressure; // Pascals
    float temp;     // Celsius
} bmp388_data_t;

bool bmp388_init(i2c_inst_t *i2c);
void bmp388_read_data(i2c_inst_t *i2c, bmp388_data_t *data);

#endif
