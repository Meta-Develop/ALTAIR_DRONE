#include "bno055.h"
#include <stdio.h>  // For debug prints if needed

static i2c_inst_t *bno_i2c;

// Register Addresses
#define BNO055_ACC_DATA_X_LSB_ADDR 0x08
#define BNO055_GYR_DATA_X_LSB_ADDR 0x14

int bno055_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    // Use timeout instead of blocking. 10ms timeout.
    // i2c_write_timeout_us returns number of bytes written (2) on success, or execution error code
    return i2c_write_timeout_us(bno_i2c, BNO055_ADDR, buf, 2, false, 10000);
}

bool bno055_set_high_speed_mode(void) {
    // 1. Enter CONFIG Mode to allow Page Switching and Configuration
    if (bno055_write_register(BNO055_OPR_MODE_ADDR, OPMODE_CONFIG) < 2) return false;
    sleep_ms(20); // Switch time config -> config is fast, but mode switch takes 19ms +

    // 2. Switch to Page 1
    if (bno055_write_register(BNO055_PAGE_ID_ADDR, 1) < 2) return false;
    sleep_ms(10); // Small delay for safety

    // 3. Configure Accelerometer
    if (bno055_write_register(BNO055_ACC_CONFIG_ADDR, 0x1D) < 2) return false;

    // 4. Configure Gyroscope
    if (bno055_write_register(BNO055_GYR_CONFIG_0_ADDR, 0x00) < 2) return false;

    // 5. Switch back to Page 0
    if (bno055_write_register(BNO055_PAGE_ID_ADDR, 0) < 2) return false;
    sleep_ms(10);

    // 6. Enter AMG Mode (Non-Fusion)
    if (bno055_write_register(BNO055_OPR_MODE_ADDR, OPMODE_AMG) < 2) return false;
    sleep_ms(20); // Wait for mode switch

    return true;
}

bool bno055_init(i2c_inst_t *i2c_instance, uint sda, uint scl) {
    bno_i2c = i2c_instance;
    
    // Initialize I2C - 400kHz recommended for high speed
    i2c_init(bno_i2c, 400 * 1000); 
    
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda); // Internal pullups
    gpio_pull_up(scl);

    // Perform specific configuration
    return bno055_set_high_speed_mode();
}

bool bno055_read_raw(bno055_data_t *data) {
    uint8_t buffer[6];
    int ret;

    // Read Accelerometer (6 bytes)
    uint8_t reg_acc = BNO055_ACC_DATA_X_LSB_ADDR;
    // Write register address (no stop)
    ret = i2c_write_timeout_us(bno_i2c, BNO055_ADDR, &reg_acc, 1, true, 2000);
    if (ret < 0) return false;
    // Read data
    ret = i2c_read_timeout_us(bno_i2c, BNO055_ADDR, buffer, 6, false, 2000);
    if (ret != 6) return false;

    data->accel_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->accel_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->accel_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Read Gyroscope (6 bytes)
    uint8_t reg_gyr = BNO055_GYR_DATA_X_LSB_ADDR;
    ret = i2c_write_timeout_us(bno_i2c, BNO055_ADDR, &reg_gyr, 1, true, 2000);
    if (ret < 0) return false;
    
    ret = i2c_read_timeout_us(bno_i2c, BNO055_ADDR, buffer, 6, false, 2000);
    if (ret != 6) return false;

    data->gyro_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->gyro_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->gyro_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}
