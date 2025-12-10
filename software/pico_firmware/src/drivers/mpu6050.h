#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C Address (AD0 = GND)
#define MPU6050_ADDR 0x68

// Registers
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define REG_ACCEL_XOUT_H 0x3B // Start of burst read (Accel -> Temp -> Gyro)

// Scales
#define ACCEL_RANGE_4G   0x08 // +/- 4g (FS_SEL=1)
#define ACCEL_RANGE_16G  0x18 // +/- 16g (FS_SEL=3)
#define GYRO_RANGE_2000  0x18 // +/- 2000dps (FS_SEL=3)
#define DLPF_42HZ        0x03 // ~42Hz Bandwidth

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_data_t;

// Prototypes
// Prototypes
// Prototypes
void mpu6050_init(i2c_inst_t *i2c);
bool mpu6050_read_burst(i2c_inst_t *i2c, mpu6050_data_t *data);

#endif
