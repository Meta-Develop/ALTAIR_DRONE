#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Configuration
#define ISM330_SPI_PORT spi1
#define ISM330_PIN_SCK  10
#define ISM330_PIN_MOSI 11
#define ISM330_PIN_MISO 12
#define ISM330_PIN_CS   13
#define ISM330_SPI_SPEED (10 * 1000 * 1000) // 10MHz

typedef struct {
    float accel[3]; // m/s^2
    float gyro[3];  // rad/s
    float temp;     // Celsius
} ism330_data_t;

bool ism330_init(void);
void ism330_read_data(ism330_data_t *data);

#endif
