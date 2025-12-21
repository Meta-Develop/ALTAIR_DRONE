#include "mmc5983ma.h"
#include <stdio.h>

// TODO: Implement Real I2C Driver for MMC5983MA
// Current implementation is a stub to match hardware constraints (I2C) vs legacy code (SPI).

bool mmc5983_init(spi_inst_t *spi, uint cs_pin) {
    // SPI Config
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // Deselect
    sleep_ms(10);
    
    // Read Product ID (0x00)
    // Bit 7 = 1 (Read)
    uint8_t tx[2] = {0x00 | 0x80, 0x00};
    uint8_t rx[2] = {0, 0};
    
    gpio_put(cs_pin, 0);
    spi_write_read_blocking(spi, tx, rx, 2);
    gpio_put(cs_pin, 1);

    printf("[MMC5983] Product ID: 0x%02X (Expect 0x30)\n", rx[1]);

    if(rx[1] != 0x30) {
        printf("[MMC5983] Init Failed!\n");
        return false;
    }
    
    printf("[MMC5983] Init Success.\n");
    return true; 
}

void mmc5983_read_data(spi_inst_t *spi, uint cs_pin, mmc5983_data_t *data) {
    // Placeholder for data reading
    data->mag[0] = 0.0f;
    data->mag[1] = 0.0f;
    data->mag[2] = 0.0f;
}
