/*
 * board_config.h - Pin Definitions and Constants for Pico 2A Sensor Hub
 * 
 * ============================================================================
 * HARDWARE LAYOUT (Prototype V1):
 * ============================================================================
 * This file centralizes all pin definitions for the Pico 2A sensor board.
 * Modify these values if rewiring is required.
 * 
 * ============================================================================
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "hardware/spi.h"
#include "hardware/i2c.h"

// =============================================================================
// SPI Slave to RPi4 (PIO-based on SPI0 Pins)
// =============================================================================
#define PIN_SPI_SLAVE_MOSI  16  // GP16 - Data from RPi (unused, we send only)
#define PIN_SPI_SLAVE_CS    17  // GP17 - Chip Select from RPi (Active Low)
#define PIN_SPI_SLAVE_SCK   18  // GP18 - Clock from RPi
#define PIN_SPI_SLAVE_MISO  19  // GP19 - Data to RPi

// =============================================================================
// Sensor SPI Bus (Hardware SPI1)
// =============================================================================
#define SPI_SENS_PORT       spi1
#define SPI_SENS_BAUDRATE   5000000  // 5 MHz

#define PIN_SENS_SCK        10  // GP10 - SPI1 SCK
#define PIN_SENS_MOSI       11  // GP11 - SPI1 TX
#define PIN_SENS_MISO       12  // GP12 - SPI1 RX
#define PIN_IMU_CS          13  // GP13 - ISM330DHCX Chip Select
#define PIN_MAG_CS          14  // GP14 - MMC5983MA Chip Select

// =============================================================================
// I2C Bus (Hardware I2C0)
// =============================================================================
#define I2C_SENS_PORT       i2c0
#define I2C_SENS_BAUDRATE   100000  // 100 kHz (Safe for internal pull-ups)

#define PIN_I2C_SDA         4   // GP4 - I2C0 SDA
#define PIN_I2C_SCL         5   // GP5 - I2C0 SCL

// I2C Device Addresses
#define BMP388_ADDR_PRIMARY   0x77
#define BMP388_ADDR_SECONDARY 0x76
#define VL53L4CX_ADDR_DEFAULT 0x29

// =============================================================================
// Status LED
// =============================================================================
#define PIN_LED             25  // Onboard LED

// =============================================================================
// PIO Configuration
// =============================================================================
#define PIO_SPI_SLAVE       pio0
#define PIO_SM_SPI_SLAVE    0   // State Machine 0

// =============================================================================
// Sensor Polling Rates (Microseconds)
// =============================================================================
#define MAG_POLL_INTERVAL_US    10000   // 100 Hz
#define BARO_POLL_INTERVAL_US   20000   // 50 Hz
#define TOF_POLL_INTERVAL_US    33000   // ~30 Hz
#define MAIN_LOOP_DELAY_US      50      // Fast polling for IMU

#endif // BOARD_CONFIG_H
