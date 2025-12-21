#include "ism330dhcx.h"
#include <string.h>
#include <stdio.h>

// SPI Read/Write helpers
static uint8_t ism330_read_reg(spi_inst_t *spi, uint cs_pin, uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00};  // Bit 7 = 1 for read
    uint8_t rx[2];
    
    gpio_put(cs_pin, 0);
    spi_write_read_blocking(spi, tx, rx, 2);
    gpio_put(cs_pin, 1);
    
    return rx[1];
}

static void ism330_write_reg(spi_inst_t *spi, uint cs_pin, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val};  // Bit 7 = 0 for write
    
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, tx, 2);
    gpio_put(cs_pin, 1);
}

static void ism330_read_burst(spi_inst_t *spi, uint cs_pin, uint8_t start_reg, uint8_t *buf, size_t len) {
    uint8_t cmd = start_reg | 0x80;  // Read + auto-increment
    
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buf, len);
    gpio_put(cs_pin, 1);
}

bool ism330_init(spi_inst_t *spi, uint cs_pin) {
    // CS Pin Setup
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);  // Deselect
    sleep_ms(10);  // Boot time
    
    // Read WHO_AM_I
    uint8_t who = ism330_read_reg(spi, cs_pin, ISM330_WHO_AM_I);
    printf("[ISM330] WHO_AM_I: 0x%02X (Expected 0x%02X)\n", who, ISM330_WHO_AM_I_VAL);
    
    if (who != ISM330_WHO_AM_I_VAL) {
        printf("[ISM330] Init FAILED - Wrong WHO_AM_I!\n");
        return false;
    }
    
    // Software Reset (CTRL3_C bit 0)
    ism330_write_reg(spi, cs_pin, ISM330_CTRL3_C, 0x01);
    sleep_ms(10);
    
    // Configure Accelerometer: 1.66kHz ODR, ±16g
    // CTRL1_XL = ODR[7:4] + FS[3:2] + LPF2_XL_EN[1] + 0
    ism330_write_reg(spi, cs_pin, ISM330_CTRL1_XL, ISM330_ODR_1660HZ | ISM330_FS_XL_16G);
    
    // Configure Gyroscope: 1.66kHz ODR, ±2000dps
    // CTRL2_G = ODR[7:4] + FS[3:2] + FS_125[1] + 0
    ism330_write_reg(spi, cs_pin, ISM330_CTRL2_G, ISM330_ODR_1660HZ | ISM330_FS_G_2000DPS);
    
    // CTRL3_C: Block Data Update (BDU) enable, auto-increment
    ism330_write_reg(spi, cs_pin, ISM330_CTRL3_C, 0x44);  // BDU + IF_INC
    
    printf("[ISM330] Init SUCCESS - 1.66kHz ODR, ±16g, ±2000dps\n");
    return true;
}

void ism330_read_data(spi_inst_t *spi, uint cs_pin, ism330_data_t *data) {
    // Read 14 bytes: Temp(2) + Gyro(6) + Accel(6) from 0x20-0x2D
    uint8_t buf[14];
    ism330_read_burst(spi, cs_pin, ISM330_OUT_TEMP_L, buf, 14);
    
    // Temperature (offset 0-1)
    int16_t temp_raw = (int16_t)(buf[1] << 8 | buf[0]);
    data->temp = 25.0f + (float)temp_raw / 256.0f;
    
    // Gyroscope (offset 2-7): X, Y, Z
    int16_t gx = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gy = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t gz = (int16_t)(buf[7] << 8 | buf[6]);
    
    data->gyro[0] = (float)gx * ISM330_SENS_2000DPS;
    data->gyro[1] = (float)gy * ISM330_SENS_2000DPS;
    data->gyro[2] = (float)gz * ISM330_SENS_2000DPS;
    
    // Accelerometer (offset 8-13): X, Y, Z
    int16_t ax = (int16_t)(buf[9] << 8 | buf[8]);
    int16_t ay = (int16_t)(buf[11] << 8 | buf[10]);
    int16_t az = (int16_t)(buf[13] << 8 | buf[12]);
    
    data->accel[0] = (float)ax * ISM330_SENS_16G;
    data->accel[1] = (float)ay * ISM330_SENS_16G;
    data->accel[2] = (float)az * ISM330_SENS_16G;
}

bool ism330_data_ready(spi_inst_t *spi, uint cs_pin) {
    uint8_t status = ism330_read_reg(spi, cs_pin, ISM330_STATUS_REG);
    // Bit 0 = XLDA (Accel ready), Bit 1 = GDA (Gyro ready)
    return (status & 0x03) == 0x03;
}
