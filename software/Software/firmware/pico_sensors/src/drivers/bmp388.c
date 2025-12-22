#include "bmp388.h"
#include <stdio.h>
#include <math.h>

// Static calibration coefficients (read once during init)
static bmp388_calib_t calib;
static float PAR_T1, PAR_T2, PAR_T3;
static float PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6, PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;
static float t_lin;  // Cached linearized temperature for pressure compensation

// I2C helpers
static bool bmp388_read_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t *buf, size_t len) {
    int ret = i2c_write_blocking(i2c, BMP388_ADDR, &reg, 1, true);
    if (ret < 0) return false;
    ret = i2c_read_blocking(i2c, BMP388_ADDR, buf, len, false);
    return ret >= 0;
}

static bool bmp388_write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int ret = i2c_write_blocking(i2c, BMP388_ADDR, buf, 2, false);
    return ret >= 0;
}

// Read calibration coefficients from NVM (registers 0x31-0x45)
static bool bmp388_read_calibration(i2c_inst_t *i2c) {
    uint8_t buf[21];
    if (!bmp388_read_reg(i2c, BMP388_CALIB_DATA, buf, 21)) {
        printf("[BMP388] Failed to read calibration data\n");
        return false;
    }
    
    // Parse according to datasheet (little endian)
    calib.NVM_PAR_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    calib.NVM_PAR_T2 = (uint16_t)(buf[3] << 8 | buf[2]);
    calib.NVM_PAR_T3 = (int8_t)buf[4];
    calib.NVM_PAR_P1 = (int16_t)(buf[6] << 8 | buf[5]);
    calib.NVM_PAR_P2 = (int16_t)(buf[8] << 8 | buf[7]);
    calib.NVM_PAR_P3 = (int8_t)buf[9];
    calib.NVM_PAR_P4 = (int8_t)buf[10];
    calib.NVM_PAR_P5 = (uint16_t)(buf[12] << 8 | buf[11]);
    calib.NVM_PAR_P6 = (uint16_t)(buf[14] << 8 | buf[13]);
    calib.NVM_PAR_P7 = (int8_t)buf[15];
    calib.NVM_PAR_P8 = (int8_t)buf[16];
    calib.NVM_PAR_P9 = (int16_t)(buf[18] << 8 | buf[17]);
    calib.NVM_PAR_P10 = (int8_t)buf[19];
    calib.NVM_PAR_P11 = (int8_t)buf[20];
    
    // Convert to floating point coefficients (from datasheet)
    PAR_T1 = (float)calib.NVM_PAR_T1 / powf(2, -8);
    PAR_T2 = (float)calib.NVM_PAR_T2 / powf(2, 30);
    PAR_T3 = (float)calib.NVM_PAR_T3 / powf(2, 48);
    
    PAR_P1 = ((float)calib.NVM_PAR_P1 - powf(2, 14)) / powf(2, 20);
    PAR_P2 = ((float)calib.NVM_PAR_P2 - powf(2, 14)) / powf(2, 29);
    PAR_P3 = (float)calib.NVM_PAR_P3 / powf(2, 32);
    PAR_P4 = (float)calib.NVM_PAR_P4 / powf(2, 37);
    PAR_P5 = (float)calib.NVM_PAR_P5 / powf(2, -3);
    PAR_P6 = (float)calib.NVM_PAR_P6 / powf(2, 6);
    PAR_P7 = (float)calib.NVM_PAR_P7 / powf(2, 8);
    PAR_P8 = (float)calib.NVM_PAR_P8 / powf(2, 15);
    PAR_P9 = (float)calib.NVM_PAR_P9 / powf(2, 48);
    PAR_P10 = (float)calib.NVM_PAR_P10 / powf(2, 48);
    PAR_P11 = (float)calib.NVM_PAR_P11 / powf(2, 65);
    
    return true;
}

// Compensation algorithm from Bosch datasheet
static float compensate_temperature(uint32_t raw_temp) {
    float partial_data1 = (float)raw_temp - PAR_T1;
    float partial_data2 = partial_data1 * PAR_T2;
    t_lin = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
    return t_lin;
}

static float compensate_pressure(uint32_t raw_press) {
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;
    
    partial_data1 = PAR_P6 * t_lin;
    partial_data2 = PAR_P7 * (t_lin * t_lin);
    partial_data3 = PAR_P8 * (t_lin * t_lin * t_lin);
    partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = PAR_P2 * t_lin;
    partial_data2 = PAR_P3 * (t_lin * t_lin);
    partial_data3 = PAR_P4 * (t_lin * t_lin * t_lin);
    partial_out2 = (float)raw_press * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = PAR_P9 + PAR_P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * PAR_P11;
    
    return partial_out1 + partial_out2 + partial_data4;
}

bool bmp388_init(i2c_inst_t *i2c) {
    uint8_t chip_id;
    
    // Read and verify chip ID
    if (!bmp388_read_reg(i2c, BMP388_CHIP_ID, &chip_id, 1)) {
        printf("[BMP388] I2C Read Failed\n");
        return false;
    }
    
    printf("[BMP388] Chip ID: 0x%02X (Expected 0x%02X)\n", chip_id, BMP388_CHIP_ID_VAL);
    if (chip_id != BMP388_CHIP_ID_VAL) {
        printf("[BMP388] Init FAILED - Wrong Chip ID\n");
        return false;
    }
    
    // Read calibration coefficients
    if (!bmp388_read_calibration(i2c)) {
        return false;
    }
    
    // Configure sensor:
    // OSR: Pressure x8, Temperature x1 = 0x03
    bmp388_write_reg(i2c, BMP388_OSR, 0x03);
    
    // ODR: 50Hz = 0x02
    bmp388_write_reg(i2c, BMP388_ODR, 0x02);
    
    // IIR Filter: coefficient 3 = 0x04
    bmp388_write_reg(i2c, BMP388_CONFIG, 0x04);
    
    // Power: Enable pressure and temp, Normal mode = 0x33
    bmp388_write_reg(i2c, BMP388_PWR_CTRL, 0x33);
    
    printf("[BMP388] Init SUCCESS\n");
    return true;
}

void bmp388_read_data(i2c_inst_t *i2c, bmp388_data_t *data) {
    uint8_t buf[6];
    
    if (!bmp388_read_reg(i2c, BMP388_DATA_0, buf, 6)) {
        data->pressure = 0;
        data->temp = 0;
        return;
    }
    
    // Parse 24-bit raw values (little endian)
    uint32_t raw_press = (uint32_t)buf[2] << 16 | (uint32_t)buf[1] << 8 | buf[0];
    uint32_t raw_temp = (uint32_t)buf[5] << 16 | (uint32_t)buf[4] << 8 | buf[3];
    
    // Compensate (must do temperature first to set t_lin)
    data->temp = compensate_temperature(raw_temp);
    data->pressure = compensate_pressure(raw_press);
}
