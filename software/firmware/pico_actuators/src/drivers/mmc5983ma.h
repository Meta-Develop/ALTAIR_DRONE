#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Configuration (Shared with ISM330)
#define MMC5983_SPI_PORT spi1
#define MMC5983_PIN_CS   19  // Separate CS from ISM330
#define MMC5983_SPI_SPEED (10 * 1000 * 1000) // 10MHz

typedef struct {
    float mag[3]; // Gauss or uT
} mmc5983_data_t;

bool mmc5983_init(void);
void mmc5983_read_data(mmc5983_data_t *data);

#endif
