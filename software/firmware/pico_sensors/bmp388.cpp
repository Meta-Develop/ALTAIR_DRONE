#include "bmp388.h"
#include <stdio.h>
#include <math.h>

// Helper to combine bytes
static uint16_t concat16(uint8_t msb, uint8_t lsb) {
    return (uint16_t)msb << 8 | lsb; // Wait, Little Endian?
    // BMP388 sends LSB first for 16-bit? 
    // Datasheet: "All the calibration coefficients are 16-bit or 8-bit signed/unsigned integers... stored in little endian format."
    // So buffer[0] is LSB, buffer[1] is MSB.
}

BMP388::BMP388(i2c_inst_t* i2c, uint8_t addr) : _i2c(i2c), _addr(addr) {}

void BMP388::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(_i2c, _addr, buf, 2, false);
}

uint8_t BMP388::read_reg(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(_i2c, _addr, &reg, 1, true);
    i2c_read_blocking(_i2c, _addr, &val, 1, false);
    return val;
}

void BMP388::read_regs(uint8_t reg, uint8_t* buf, int len) {
    i2c_write_blocking(_i2c, _addr, &reg, 1, true);
    i2c_read_blocking(_i2c, _addr, buf, len, false);
}

void BMP388::read_calibration() {
    uint8_t cal[21];
    read_regs(BMP388_REG_CALIB_DATA, cal, 21);
    
    // Parse Little Endian
    _calib.T1 = (uint16_t)(cal[1] << 8 | cal[0]);
    _calib.T2 = (uint16_t)(cal[3] << 8 | cal[2]);
    _calib.T3 = (int8_t)cal[4];
    _calib.P1 = (int16_t)(cal[6] << 8 | cal[5]);
    _calib.P2 = (int16_t)(cal[8] << 8 | cal[7]);
    _calib.P3 = (int8_t)cal[9];
    _calib.P4 = (int8_t)cal[10];
    _calib.P5 = (uint16_t)(cal[12] << 8 | cal[11]);
    _calib.P6 = (uint16_t)(cal[14] << 8 | cal[13]);
    _calib.P7 = (int8_t)cal[15];
    _calib.P8 = (int8_t)cal[16];
    _calib.P9 = (int16_t)(cal[18] << 8 | cal[17]);
    _calib.P10 = (int8_t)cal[19];
    _calib.P11 = (int8_t)cal[20];
}

bool BMP388::init() {
    uint8_t id = read_reg(BMP388_REG_CHIP_ID);
    if (id != BMP388_CHIP_ID) {
        printf("BMP388 ID mismatch: 0x%02X\n", id);
        return false;
    }
    
    read_calibration();
    
    // Reset? maybe later.
    
    // Config PWR_CTRL: Enable Press(bit 0), Temp(bit 1), Mode=Normal(bit 5,4 = 11)
    // 0011 0011 = 0x33
    write_reg(BMP388_REG_PWR_CTRL, 0x33);
    
    // OSR: Standard resolution. P_OSR=x8(011), T_OSR=x1(000)
    // Register: bits 2:0 = P_OSR, bits 5:3 = T_OSR
    // x8 Pressure = 011 -> 3
    // x1 Temp = 000 -> 0
    // Val = 0000 0011 = 0x03
    write_reg(BMP388_REG_OSR, 0x03); 
    
    // ODR: 50Hz. Prescaler 0x02?
    // 0x00 = 200Hz, 0x01 = 100Hz, 0x02 = 50Hz
    write_reg(BMP388_REG_ODR, 0x02);
    
    // Config: IIR Filter? Set to coeff 3 (010) -> bits 3:1
    // Let's leave filter off for now (0x00)
    write_reg(BMP388_REG_CONFIG, 0x00);
    
    return true;
}

// Compensation variables
// Using float version of Bosch formula from datasheet/driver
float BMP388::compensate_temperature(uint32_t uncomp_temp) {
    float partial_data1;
    float partial_data2;
    
    partial_data1 = (float)(uncomp_temp - _calib.T1);
    partial_data2 = (float)(partial_data1 * _calib.T2);
    
    // Temperature in deg C
    return partial_data2 + (partial_data1 * partial_data1) * _calib.T3; 
}

float BMP388::compensate_pressure(uint32_t uncomp_press, float t_lin) {
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;
    
    partial_data1 = _calib.P6 * t_lin;
    partial_data2 = _calib.P7 * (t_lin * t_lin);
    partial_data3 = _calib.P8 * (t_lin * t_lin * t_lin);
    partial_out1 = _calib.P5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = _calib.P2 * t_lin;
    partial_data2 = _calib.P3 * (t_lin * t_lin);
    partial_data3 = _calib.P4 * (t_lin * t_lin * t_lin);
    partial_out2 = (float)uncomp_press * (_calib.P1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = _calib.P9 + _calib.P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * _calib.P11;
    
    float comp_press = partial_out1 + partial_out2 + partial_data4;
    return comp_press; // Pascal
}

bool BMP388::read_data(float* pressure, float* temperature) {
    uint8_t data[6];
    read_regs(BMP388_REG_DATA_0, data, 6);
    // Data 0-2: Pressure (XLSB, LSB, MSB)
    // Data 3-5: Temp (XLSB, LSB, MSB)
    
    uint32_t adc_p = (data[2] << 16) | (data[1] << 8) | data[0];
    uint32_t adc_t = (data[5] << 16) | (data[4] << 8) | data[3];
    
    *temperature = compensate_temperature(adc_t);
    *pressure = compensate_pressure(adc_p, *temperature);
    return true;
}
