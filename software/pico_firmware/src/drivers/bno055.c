#include "bno055.h"

static i2c_inst_t *bno_i2c;

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_ACC_DATA_X_LSB_ADDR 0x08
#define BNO055_GYR_DATA_X_LSB_ADDR 0x14
#define OPMODE_AMG 0x07
#define OPMODE_CONFIG 0x00

static void write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    i2c_write_blocking(bno_i2c, BNO055_ADDR, buf, 2, false);
}

void bno055_init(i2c_inst_t *i2c_instance, uint sda, uint scl) {
    bno_i2c = i2c_instance;
    i2c_init(bno_i2c, 400 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // Reset routine or simple config
    write_reg(BNO055_OPR_MODE_ADDR, OPMODE_CONFIG);
    sleep_ms(20);
    write_reg(BNO055_OPR_MODE_ADDR, OPMODE_AMG); // Accel Gyro Mag (Raw)
    sleep_ms(20);
}

bool bno055_read_raw(bno055_data_t *data) {
    uint8_t buffer[6];
    
    // Read Accel
    uint8_t reg = BNO055_ACC_DATA_X_LSB_ADDR;
    i2c_write_blocking(bno_i2c, BNO055_ADDR, &reg, 1, true);
    if (i2c_read_blocking(bno_i2c, BNO055_ADDR, buffer, 6, false) != 6) return false;
    
    data->accel_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->accel_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->accel_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Read Gyro
    reg = BNO055_GYR_DATA_X_LSB_ADDR;
    i2c_write_blocking(bno_i2c, BNO055_ADDR, &reg, 1, true);
    if (i2c_read_blocking(bno_i2c, BNO055_ADDR, buffer, 6, false) != 6) return false;

    data->gyro_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->gyro_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->gyro_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}
