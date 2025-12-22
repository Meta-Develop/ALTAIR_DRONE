#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP388_ADDR 0x77  // Or 0x76 if SDO to GND

// BMP388 Register Map
#define BMP388_CHIP_ID      0x00
#define BMP388_ERR_REG      0x02
#define BMP388_STATUS       0x03
#define BMP388_DATA_0       0x04   // pressure_xlsb
#define BMP388_DATA_1       0x05   // pressure_lsb
#define BMP388_DATA_2       0x06   // pressure_msb
#define BMP388_DATA_3       0x07   // temp_xlsb
#define BMP388_DATA_4       0x08   // temp_lsb
#define BMP388_DATA_5       0x09   // temp_msb
#define BMP388_PWR_CTRL     0x1B
#define BMP388_OSR          0x1C   // Oversampling
#define BMP388_ODR          0x1D   // Output Data Rate
#define BMP388_CONFIG       0x1F   // IIR Filter
#define BMP388_CALIB_DATA   0x31   // Calibration coefficients start

// Expected Chip ID
#define BMP388_CHIP_ID_VAL  0x50

// Calibration structure (21 bytes from 0x31-0x45)
typedef struct {
    uint16_t NVM_PAR_T1;
    uint16_t NVM_PAR_T2;
    int8_t   NVM_PAR_T3;
    int16_t  NVM_PAR_P1;
    int16_t  NVM_PAR_P2;
    int8_t   NVM_PAR_P3;
    int8_t   NVM_PAR_P4;
    uint16_t NVM_PAR_P5;
    uint16_t NVM_PAR_P6;
    int8_t   NVM_PAR_P7;
    int8_t   NVM_PAR_P8;
    int16_t  NVM_PAR_P9;
    int8_t   NVM_PAR_P10;
    int8_t   NVM_PAR_P11;
} bmp388_calib_t;

typedef struct {
    float pressure;  // Pascals
    float temp;      // Celsius
} bmp388_data_t;

// Initialize BMP388 sensor (reads calibration, configures)
bool bmp388_init(i2c_inst_t *i2c);

// Read compensated pressure and temperature
void bmp388_read_data(i2c_inst_t *i2c, bmp388_data_t *data);

#endif
