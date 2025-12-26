#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// MMC5983MA Register Definitions
#define MMC5983_XOUT0       0x00  // X[17:10]
#define MMC5983_XOUT1       0x01  // X[9:2]
#define MMC5983_YOUT0       0x02  // Y[17:10]
#define MMC5983_YOUT1       0x03  // Y[9:2]
#define MMC5983_ZOUT0       0x04  // Z[17:10]
#define MMC5983_ZOUT1       0x05  // Z[9:2]
#define MMC5983_XYZOUT2     0x06  // X[1:0], Y[1:0], Z[1:0]
#define MMC5983_TOUT        0x07  // Temperature
#define MMC5983_STATUS      0x08
#define MMC5983_CTRL0       0x09
#define MMC5983_CTRL1       0x0A
#define MMC5983_CTRL2       0x0B
#define MMC5983_CTRL3       0x0C
#define MMC5983_PRODUCT_ID  0x2F

// Continuous Mode Macros
#define MMC5983_CMD_TM_M       0x01
#define MMC5983_CMD_SW_RST     0x80

#define MMC5983_CM_FREQ_100HZ  0x05
#define MMC5983_CM_FREQ_200HZ  0x06
#define MMC5983_CM_FREQ_1000HZ 0x07 // Check datasheet validity for 1000Hz (typ 600Hz BW)

#define MMC5983_BW_100HZ       0x00
#define MMC5983_BW_200HZ       0x01
#define MMC5983_BW_400HZ       0x02
#define MMC5983_BW_800HZ       0x03

// Expected Product ID
#define MMC5983_PRODUCT_ID_VAL  0x30

// Sensitivity: ±8G range, 18-bit (131072 counts for 16G span)
// Offset midpoint = 131072 (unsigned output)
// 1 LSB = 16G / 262144 = 0.000061035 Gauss
// Convert to uT: 1 Gauss = 100 uT
#define MMC5983_SENS_UT  (0.0061035f)  // uT per LSB (18-bit mode)
#define MMC5983_OFFSET   131072

typedef struct {
    float mag[3];  // Magnetic field in uT (micro-Tesla)
} mmc5983_data_t;

// Initialize MMC5983MA sensor (returns true if Product ID matches)
bool mmc5983_init(spi_inst_t *spi, uint cs_pin);

// Trigger a single measurement (must call before read)
void mmc5983_trigger_measurement(spi_inst_t *spi, uint cs_pin);

// Check if measurement is complete
bool mmc5983_data_ready(spi_inst_t *spi, uint cs_pin);

// Read magnetic field data (X, Y, Z) in uT
void mmc5983_read_data(spi_inst_t *spi, uint cs_pin, mmc5983_data_t *data);

#endif
