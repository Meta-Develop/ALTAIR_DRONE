#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

typedef struct {
    float mag[3]; // Gauss
} mmc5983_data_t;

bool mmc5983_init(spi_inst_t *spi, uint cs_pin);
void mmc5983_read_data(spi_inst_t *spi, uint cs_pin, mmc5983_data_t *data);

#endif
