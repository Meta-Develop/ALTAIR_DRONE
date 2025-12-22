#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// ISM330DHCX Register Definitions
#define ISM330_WHO_AM_I     0x0F
#define ISM330_CTRL1_XL     0x10  // Accelerometer control
#define ISM330_CTRL2_G      0x11  // Gyroscope control
#define ISM330_CTRL3_C      0x12  // Control register 3
#define ISM330_CTRL4_C      0x13  // Control register 4
#define ISM330_STATUS_REG   0x1E
#define ISM330_OUT_TEMP_L   0x20
#define ISM330_OUTX_L_G     0x22  // Gyro data start
#define ISM330_OUTX_L_A     0x28  // Accel data start

// WHO_AM_I expected value
#define ISM330_WHO_AM_I_VAL 0x6B

// ODR settings (bits 7:4 of CTRL1_XL / CTRL2_G)
#define ISM330_ODR_1660HZ   0x80  // 1.66kHz (closest to 1kHz with margin)
#define ISM330_ODR_833HZ    0x70  // 833Hz

// Full scale settings
#define ISM330_FS_XL_16G    0x04  // ±16g (bits 3:2)
#define ISM330_FS_G_2000DPS 0x0C  // ±2000dps (bits 3:2)

// Sensitivity values
#define ISM330_SENS_16G     (0.488f / 1000.0f * 9.81f)  // mg/LSB -> m/s²
#define ISM330_SENS_2000DPS (70.0f / 1000.0f * 0.0174533f)  // mdps/LSB -> rad/s

typedef struct {
    float accel[3]; // m/s^2
    float gyro[3];  // rad/s
    float temp;     // Celsius
} ism330_data_t;

// Initialize ISM330DHCX sensor
// Returns true if WHO_AM_I matches expected value
bool ism330_init(spi_inst_t *spi, uint cs_pin);

// Read all sensor data (accel, gyro, temp) in one burst
void ism330_read_data(spi_inst_t *spi, uint cs_pin, ism330_data_t *data);

// Check if new data is available
bool ism330_data_ready(spi_inst_t *spi, uint cs_pin);

#endif
