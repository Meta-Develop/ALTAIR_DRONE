#include "mpu6050.h"
#include <stdio.h>

static void mpu6050_write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = val;
    i2c_write_blocking(i2c, MPU6050_ADDR, buffer, 2, false);
}

void mpu6050_init(i2c_inst_t *i2c) {
    // Expected that i2c init and pin config is done in main
    
    // Wakeup (Clear SLEEP bit)
    mpu6050_write_reg(i2c, REG_PWR_MGMT_1, 0x00);
    sleep_ms(50);
    
    // Config (DLPF=3: ~42Hz Bandwidth)
    // Good for filtering noise, decent delay approx 4.8ms
    mpu6050_write_reg(i2c, REG_CONFIG, DLPF_42HZ);
    
    // Gyro +/- 2000 dps
    mpu6050_write_reg(i2c, REG_GYRO_CONFIG, GYRO_RANGE_2000);
    
    // Accel +/- 16g
    mpu6050_write_reg(i2c, REG_ACCEL_CONFIG, ACCEL_RANGE_16G);
}

bool mpu6050_read_burst(i2c_inst_t *i2c, mpu6050_data_t *data) {
    uint8_t reg = REG_ACCEL_XOUT_H;
    int ret;
    
    // Write Start Addr
    ret = i2c_write_blocking(i2c, MPU6050_ADDR, &reg, 1, true); // true = nostop
    if (ret == PICO_ERROR_GENERIC) return false;
    
    // Read 14 bytes
    uint8_t buffer[14];
    ret = i2c_read_blocking(i2c, MPU6050_ADDR, buffer, 14, false);
    if (ret == PICO_ERROR_GENERIC) return false;
    
    // Parse
    data->ax = (buffer[0] << 8) | buffer[1];
    data->ay = (buffer[2] << 8) | buffer[3];
    data->az = (buffer[4] << 8) | buffer[5];
    data->temp = (buffer[6] << 8) | buffer[7];
    data->gx = (buffer[8] << 8) | buffer[9];
    data->gy = (buffer[10] << 8) | buffer[11];
    data->gz = (buffer[12] << 8) | buffer[13];
    
    return true;
}
