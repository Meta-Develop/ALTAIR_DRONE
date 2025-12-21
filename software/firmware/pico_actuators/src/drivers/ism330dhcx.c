#include "ism330dhcx.h"
#include <stdio.h>

// Registers
#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10  // Accel control
#define REG_CTRL2_G   0x11  // Gyro control
#define REG_CTRL3_C   0x12  // Control 3
#define REG_OUT_TEMP_L 0x20

// SPI helpers
static inline void cs_select(void) {
    gpio_put(ISM330_PIN_CS, 0);
}

static inline void cs_deselect(void) {
    gpio_put(ISM330_PIN_CS, 1);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0}; // Read = MSB set
    uint8_t rx[2];
    cs_select();
    spi_write_read_blocking(ISM330_SPI_PORT, tx, rx, 2);
    cs_deselect();
    return rx[1];
}

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val}; // Write = MSB clear
    cs_select();
    spi_write_blocking(ISM330_SPI_PORT, tx, 2);
    cs_deselect();
}

bool ism330_init(void) {
    // Initialize SPI pins
    spi_init(ISM330_SPI_PORT, ISM330_SPI_SPEED);
    gpio_set_function(ISM330_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ISM330_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ISM330_PIN_MISO, GPIO_FUNC_SPI);
    
    // CS as GPIO output
    gpio_init(ISM330_PIN_CS);
    gpio_set_dir(ISM330_PIN_CS, GPIO_OUT);
    gpio_put(ISM330_PIN_CS, 1); // Deselect
    
    sleep_ms(10); // Boot time
    
    // Check WHO_AM_I (expected 0x6B)
    uint8_t id = read_reg(REG_WHO_AM_I);
    if (id != 0x6B) {
        printf("ISM330 ID mismatch: 0x%02X\n", id);
        return false;
    }
    
    // Software reset
    write_reg(REG_CTRL3_C, 0x01);
    sleep_ms(10);
    
    // Accel: 6.66kHz ODR, +/-4g (0xA8)
    write_reg(REG_CTRL1_XL, 0xA8);
    
    // Gyro: 6.66kHz ODR, 2000dps (0xAC)
    write_reg(REG_CTRL2_G, 0xAC);
    
    return true;
}

void ism330_read_data(ism330_data_t *data) {
    // Burst read 14 bytes: Temp(2) + Gyro(6) + Accel(6)
    uint8_t tx[15] = {REG_OUT_TEMP_L | 0x80}; // Read starting at 0x20
    uint8_t rx[15];
    
    cs_select();
    spi_write_read_blocking(ISM330_SPI_PORT, tx, rx, 15);
    cs_deselect();
    
    // Parse (rx[0] is dummy from address byte)
    int16_t raw_temp = (rx[2] << 8) | rx[1];
    int16_t raw_gx = (rx[4] << 8) | rx[3];
    int16_t raw_gy = (rx[6] << 8) | rx[5];
    int16_t raw_gz = (rx[8] << 8) | rx[7];
    int16_t raw_ax = (rx[10] << 8) | rx[9];
    int16_t raw_ay = (rx[12] << 8) | rx[11];
    int16_t raw_az = (rx[14] << 8) | rx[13];
    
    // Scale: +/-4g = 0.122 mg/LSB, +/-2000dps = 70 mdps/LSB
    data->accel[0] = raw_ax * 0.122e-3f * 9.81f;
    data->accel[1] = raw_ay * 0.122e-3f * 9.81f;
    data->accel[2] = raw_az * 0.122e-3f * 9.81f;
    
    data->gyro[0] = raw_gx * 70.0e-3f * 0.0174533f;
    data->gyro[1] = raw_gy * 70.0e-3f * 0.0174533f;
    data->gyro[2] = raw_gz * 70.0e-3f * 0.0174533f;
    
    data->temp = (raw_temp / 256.0f) + 25.0f;
}
