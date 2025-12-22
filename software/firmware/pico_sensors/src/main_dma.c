/**
 * SPI SLAVE FIRMWARE - FULL SENSOR SUITE
 * - ISM330DHCX (Accel/Gyro) via SPI1
 * - MMC5983MA (Magnetometer) via SPI1
 * - BMP388 (Barometer) via I2C0
 * - VL53L4CX (ToF Distance) via I2C0
 * 
 * Packet: 30 bytes = 4 header + 12 IMU + 6 Mag + 4 Baro + 2 Distance + 2 reserved
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "spi_slave.pio.h"
#include "drivers/ism330dhcx.h" 
#include "drivers/mmc5983ma.h"
#include "drivers/bmp388.h"
#include "drivers/vl53l4cx.h"

// --- PIN CONFIGURATION ---
// SPI0 PIO Slave (to RPi4 Master)
#define PIN_MISO_SLAVE    19       // GP19 - Output to RPi4
#define PIN_CS_SLAVE      17       // GP17 - Input from RPi4
#define PIN_SCK_SLAVE     18       // GP18 - Input from RPi4
#define PIN_MOSI_SLAVE    16       // GP16 - Input from RPi4

// SPI1 Master (to 9DoF Sensors)
#define PIN_MISO_SENSOR   12       // GP12 - Input from Sensors
#define PIN_MOSI_SENSOR   11       // GP11 - Output to Sensors
#define PIN_SCK_SENSOR    10       // GP10 - Clock to Sensors
#define PIN_CS_IMU        13       // GP13 - CS to ISM330DHCX (ACS)
#define PIN_CS_MAG        14       // GP14 - CS to MMC5983MA (MCS)

// I2C0 (to Baro + ToF)
#define PIN_SDA           4        // GP4 (Physical Pin 6)
#define PIN_SCL           5        // GP5 (Physical Pin 7)

#define PIN_LED     25

// --- PACKET FORMAT (30 bytes) ---
// [0-3]:   Header AA BB CC DD (4 bytes)
// [4-15]:  IMU raw: GX(2), GY(2), GZ(2), AX(2), AY(2), AZ(2) = 12 bytes
// [16-21]: Mag raw: MX(2), MY(2), MZ(2) = 6 bytes (Big Endian)
// [22-25]: Baro: Pressure (4 bytes, raw 24-bit + 8-bit temp MSB)
// [26-27]: ToF Distance: mm (2 bytes)
// [28-29]: Reserved (2 bytes)
#define TOTAL_BYTES  30

// Globals
PIO pio = pio0;
uint sm = 0;
uint offset = 0;
int dma_tx;

uint32_t tx_buffer[TOTAL_BYTES] __attribute__((aligned(4))); 
volatile uint32_t irq_count = 0;

// Sensor Status Flags
bool imu_ok = false;
bool mag_ok = false;
bool baro_ok = false;
bool tof_ok = false;

// CS Falling Edge Interrupt Handler
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_CS_SLAVE && (events & GPIO_IRQ_EDGE_FALL)) {
        gpio_set_irq_enabled(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, false);
        gpio_xor_mask(1u << PIN_LED);
        irq_count++;
        
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(dma_tx);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset + spi_slave_offset_entry_point));
        
        // Pre-fill Header
        pio_sm_put(pio, sm, 0xAA000000); 
        pio_sm_put(pio, sm, 0xBB000000); 
        pio_sm_put(pio, sm, 0xCC000000); 
        pio_sm_put(pio, sm, 0xDD000000); 
        
        // DMA the sensor data
        dma_channel_set_trans_count(dma_tx, TOTAL_BYTES - 4, false);
        dma_channel_set_read_addr(dma_tx, tx_buffer + 4, true);
        
        busy_wait_us_32(2); 
        pio_sm_set_enabled(pio, sm, true);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    
    printf("=== Pico Full Sensor Suite ===\n");
    printf("ISM330DHCX + MMC5983MA (SPI1) + BMP388 + VL53L4CX (I2C0)\n");

    // LED Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // --- I2C0 INIT (for BMP388 + VL53L4CX) ---
    i2c_init(i2c0, 400 * 1000);  // 400kHz Fast Mode
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    printf("[MAIN] I2C0 Init: GP%d (SDA), GP%d (SCL)\n", PIN_SDA, PIN_SCL);

    // --- SPI1 MASTER INIT (for 9DoF Sensors) ---
    spi_init(spi1, 8 * 1000 * 1000);  // 8MHz
    gpio_set_function(PIN_MISO_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_SENSOR, GPIO_FUNC_SPI);
    
    // Initialize Sensors
    printf("[MAIN] Init ISM330DHCX...\n");
    imu_ok = ism330_init(spi1, PIN_CS_IMU);
    printf("[MAIN] ISM330DHCX: %s\n", imu_ok ? "OK" : "FAIL");
    
    printf("[MAIN] Init MMC5983MA...\n");
    mag_ok = mmc5983_init(spi1, PIN_CS_MAG);
    printf("[MAIN] MMC5983MA: %s\n", mag_ok ? "OK" : "FAIL");
    
    printf("[MAIN] Init BMP388...\n");
    baro_ok = bmp388_init(i2c0);
    printf("[MAIN] BMP388: %s\n", baro_ok ? "OK" : "FAIL");
    
    printf("[MAIN] Init VL53L4CX...\n");
    tof_ok = vl53_init(i2c0);
    printf("[MAIN] VL53L4CX: %s\n", tof_ok ? "OK" : "FAIL");
    if (tof_ok) {
        vl53_start_ranging(i2c0);
        printf("[MAIN] VL53L4CX ranging started\n");
    }

    // --- PIO SPI SLAVE INIT ---
    gpio_init(PIN_MISO_SLAVE); 
    
    // Buffer Init
    for (int i = 0; i < TOTAL_BYTES; i++) tx_buffer[i] = 0;
    tx_buffer[0] = 0xAA000000; 
    tx_buffer[1] = 0xBB000000; 
    tx_buffer[2] = 0xCC000000; 
    tx_buffer[3] = 0xDD000000;

    offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_init(pio, sm, offset, PIN_MISO_SLAVE);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_MISO_SLAVE, 1, true); 

    gpio_init(PIN_SCK_SLAVE);
    gpio_set_dir(PIN_SCK_SLAVE, GPIO_IN);
    gpio_init(PIN_MOSI_SLAVE);
    gpio_set_dir(PIN_MOSI_SLAVE, GPIO_IN); 

    // DMA Init
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true)); 
    dma_channel_configure(dma_tx, &c, &pio->txf[sm], tx_buffer + 4, 0, false);
    
    // CS IRQ Init
    gpio_init(PIN_CS_SLAVE);
    gpio_set_dir(PIN_CS_SLAVE, GPIO_IN);
    gpio_pull_up(PIN_CS_SLAVE); 
    gpio_set_irq_enabled_with_callback(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, true, &cs_irq_handler);
    printf("[MAIN] CS IRQ Enabled. Sensors: IMU=%d MAG=%d BARO=%d TOF=%d\n", 
           imu_ok, mag_ok, baro_ok, tof_ok);
    
    uint32_t last_rearm = 0;
    uint32_t last_imu_read = 0;
    uint32_t last_mag_trigger = 0;
    uint32_t last_baro_read = 0;
    uint32_t last_tof_read = 0;
    
    bmp388_data_t baro_data = {0};
    vl53_data_t tof_data = {0};
    
    while (true) {
        uint32_t now = time_us_32();
        
        // --- 1. Read IMU at ~1kHz ---
        if (now - last_imu_read >= 1000) {
            last_imu_read = now;
            
            if (imu_ok && ism330_data_ready(spi1, PIN_CS_IMU)) {
                uint8_t imu_buf[12];
                uint8_t cmd = 0x22 | 0x80;
                gpio_put(PIN_CS_IMU, 0);
                spi_write_blocking(spi1, &cmd, 1);
                spi_read_blocking(spi1, 0x00, imu_buf, 12);
                gpio_put(PIN_CS_IMU, 1);
                
                for (int i = 0; i < 12; i++) {
                    tx_buffer[4 + i] = ((uint32_t)imu_buf[i]) << 24;
                }
            }
        }
        
        // --- 2. Mag measurement every 10ms (100Hz) ---
        if (now - last_mag_trigger >= 10000) {
            last_mag_trigger = now;
            if (mag_ok) mmc5983_trigger_measurement(spi1, PIN_CS_MAG);
        }
        
        // --- 3. Read Mag if ready ---
        if (mag_ok && mmc5983_data_ready(spi1, PIN_CS_MAG)) {
            uint8_t mag_buf[7];
            uint8_t cmd = 0x00 | 0x80;
            gpio_put(PIN_CS_MAG, 0);
            spi_write_blocking(spi1, &cmd, 1);
            spi_read_blocking(spi1, 0x00, mag_buf, 7);
            gpio_put(PIN_CS_MAG, 1);
            
            uint16_t mx = (((uint16_t)mag_buf[0]) << 8) | mag_buf[1];
            uint16_t my = (((uint16_t)mag_buf[2]) << 8) | mag_buf[3];
            uint16_t mz = (((uint16_t)mag_buf[4]) << 8) | mag_buf[5];
            
            tx_buffer[16] = ((uint32_t)(mx >> 8)) << 24;
            tx_buffer[17] = ((uint32_t)(mx & 0xFF)) << 24;
            tx_buffer[18] = ((uint32_t)(my >> 8)) << 24;
            tx_buffer[19] = ((uint32_t)(my & 0xFF)) << 24;
            tx_buffer[20] = ((uint32_t)(mz >> 8)) << 24;
            tx_buffer[21] = ((uint32_t)(mz & 0xFF)) << 24;
        }
        
        // --- 4. Read Baro every 50ms (20Hz) ---
        if (now - last_baro_read >= 50000) {
            last_baro_read = now;
            if (baro_ok) {
                bmp388_read_data(i2c0, &baro_data);
                // Pack pressure as 4-byte float raw bits
                uint32_t p_raw = *((uint32_t*)&baro_data.pressure);
                tx_buffer[22] = (p_raw >> 24) << 24;
                tx_buffer[23] = ((p_raw >> 16) & 0xFF) << 24;
                tx_buffer[24] = ((p_raw >> 8) & 0xFF) << 24;
                tx_buffer[25] = (p_raw & 0xFF) << 24;
            }
        }
        
        // --- 5. Read ToF every 100ms (10Hz) ---
        if (now - last_tof_read >= 100000) {
            last_tof_read = now;
            if (tof_ok) {
                vl53_read_data(i2c0, &tof_data);
                tx_buffer[26] = ((tof_data.range_mm >> 8) & 0xFF) << 24;
                tx_buffer[27] = (tof_data.range_mm & 0xFF) << 24;
            }
        }

        // --- 6. Heartbeat ---
        static uint32_t last_heartbeat = 0;
        if (now - last_heartbeat > 1000000) {
            static uint32_t last_irq_count = 0;
            uint32_t current_irq = irq_count;
            printf("[MAIN] HB: CS IRQs: %d (D: %d), Rate: ~%d Hz\n", 
                   current_irq, current_irq - last_irq_count, current_irq - last_irq_count);
            last_irq_count = current_irq;
            last_heartbeat = now;
        }

        // --- 7. Re-arm CS IRQ ---
        if (now - last_rearm > 500) { 
            if (gpio_get(PIN_CS_SLAVE)) { 
                gpio_set_irq_enabled(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, true);
                last_rearm = now;
            }
        }
        sleep_us(50);
    }
}
