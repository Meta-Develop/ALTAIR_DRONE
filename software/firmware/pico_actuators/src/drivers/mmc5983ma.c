#include "mmc5983ma.h"
#include <stdio.h>

// Registers
#define REG_XOUT0     0x00
#define REG_CTRL0     0x09
#define REG_CTRL1     0x0A
#define REG_CTRL2     0x0B
#define REG_PRODUCT_ID 0x2F

// SPI helpers (CS is separate from ISM330)
static inline void cs_select(void) {
    gpio_put(MMC5983_PIN_CS, 0);
}

static inline void cs_deselect(void) {
    gpio_put(MMC5983_PIN_CS, 1);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0}; // Read = MSB set
    uint8_t rx[2];
    cs_select();
    spi_write_read_blocking(MMC5983_SPI_PORT, tx, rx, 2);
    cs_deselect();
    return rx[1];
}

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val}; // Write = MSB clear
    cs_select();
    spi_write_blocking(MMC5983_SPI_PORT, tx, 2);
    cs_deselect();
}

bool mmc5983_init(void) {
    // CS as GPIO output (SPI already initialized by ISM330)
    gpio_init(MMC5983_PIN_CS);
    gpio_set_dir(MMC5983_PIN_CS, GPIO_OUT);
    gpio_put(MMC5983_PIN_CS, 1); // Deselect
    
    sleep_ms(10);
    
    // Check Product ID (expected 0x30)
    uint8_t id = read_reg(REG_PRODUCT_ID);
    if (id != 0x30) {
        printf("MMC5983 ID mismatch: 0x%02X\n", id);
        // Continue anyway - some boards differ
    }
    
    // Set ODR 1000Hz, Continuous Mode
    write_reg(REG_CTRL2, 0x0F); // CMM_EN=1, CM_FREQ=1000Hz
    
    return true;
}

void mmc5983_read_data(mmc5983_data_t *data) {
    // Burst read 7 bytes (X0,X1,Y0,Y1,Z0,Z1,XYZ2)
    uint8_t tx[8] = {REG_XOUT0 | 0x80};
    uint8_t rx[8];
    
    cs_select();
    spi_write_read_blocking(MMC5983_SPI_PORT, tx, rx, 8);
    cs_deselect();
    
    // 18-bit resolution
    uint32_t raw_x = (rx[1] << 10) | (rx[2] << 2) | ((rx[7] & 0xC0) >> 6);
    uint32_t raw_y = (rx[3] << 10) | (rx[4] << 2) | ((rx[7] & 0x30) >> 4);
    uint32_t raw_z = (rx[5] << 10) | (rx[6] << 2) | ((rx[7] & 0x0C) >> 2);
    
    // Convert to Gauss: offset = 131072, sensitivity = 16384 LSB/Gauss
    data->mag[0] = ((float)raw_x - 131072.0f) / 16384.0f;
    data->mag[1] = ((float)raw_y - 131072.0f) / 16384.0f;
    data->mag[2] = ((float)raw_z - 131072.0f) / 16384.0f;
}
