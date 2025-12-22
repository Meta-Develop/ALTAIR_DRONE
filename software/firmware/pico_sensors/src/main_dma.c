/**
 * SPI SLAVE FIRMWARE WITH 9DoF: ISM330DHCX + MMC5983MA
 * - Reads Accel/Gyro from ISM330DHCX via SPI1 (Master)
 * - Reads Magnetometer from MMC5983MA via SPI1 (Master, separate CS)
 * - Transmits sensor data to RPi4 via PIO SPI Slave (on CS IRQ)
 * - Packet: 22 bytes = 4 header + 12 IMU (6 Gyro + 6 Accel) + 6 Mag raw bytes
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "spi_slave.pio.h"
#include "drivers/ism330dhcx.h" 
#include "drivers/mmc5983ma.h"

// --- PIN CONFIGURATION ---
// SPI0 PIO Slave (to RPi4 Master)
#define PIN_MISO_SLAVE    19       // GP19 - Output to RPi4
#define PIN_CS_SLAVE      17       // GP17 - Input from RPi4
#define PIN_SCK_SLAVE     18       // GP18 - Input from RPi4
#define PIN_MOSI_SLAVE    16       // GP16 - Input from RPi4

// SPI1 Master (to Sensors)
#define PIN_MISO_SENSOR   12       // GP12 - Input from Sensors
#define PIN_MOSI_SENSOR   11       // GP11 - Output to Sensors
#define PIN_SCK_SENSOR    10       // GP10 - Clock to Sensors
#define PIN_CS_IMU        13       // GP13 - CS to ISM330DHCX (ACS)
#define PIN_CS_MAG        14       // GP14 - CS to MMC5983MA (MCS)

#define PIN_LED     25

// --- PACKET FORMAT ---
// [0-3]: Header AA BB CC DD (4 bytes)
// [4-15]: IMU raw: GX(2), GY(2), GZ(2), AX(2), AY(2), AZ(2) = 12 bytes
// [16-21]: Mag raw: MX(2), MY(2), MZ(2) = 6 bytes
// Total: 22 bytes
#define TOTAL_BYTES  22

// Globals
PIO pio = pio0;
uint sm = 0;
uint offset = 0;
int dma_tx;

uint32_t tx_buffer[TOTAL_BYTES] __attribute__((aligned(4))); 
volatile uint32_t irq_count = 0;

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
        
        // DMA the sensor data (18 bytes = indices 4-21)
        dma_channel_set_trans_count(dma_tx, TOTAL_BYTES - 4, false);
        dma_channel_set_read_addr(dma_tx, tx_buffer + 4, true);
        
        busy_wait_us_32(2); 
        pio_sm_set_enabled(pio, sm, true);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    
    printf("=== Pico 9DoF SPI Slave: ISM330DHCX + MMC5983MA ===\n");

    // LED Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // --- SPI1 MASTER INIT (for Sensors) ---
    spi_init(spi1, 8 * 1000 * 1000);  // 8MHz
    gpio_set_function(PIN_MISO_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_SENSOR, GPIO_FUNC_SPI);
    
    // Initialize ISM330DHCX (Accel/Gyro)
    printf("[MAIN] Init ISM330DHCX...\n");
    bool imu_ok = ism330_init(spi1, PIN_CS_IMU);
    
    // Initialize MMC5983MA (Magnetometer)
    printf("[MAIN] Init MMC5983MA...\n");
    bool mag_ok = mmc5983_init(spi1, PIN_CS_MAG);
    
    printf("[MAIN] Sensors: IMU=%s, Mag=%s\n", imu_ok ? "OK" : "FAIL", mag_ok ? "OK" : "FAIL");

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
    printf("[MAIN] CS IRQ Enabled. Waiting for RPi4...\n");
    
    uint32_t last_rearm = 0;
    uint32_t last_sensor_read = 0;
    uint32_t last_mag_trigger = 0;
    
    while (true) {
        uint32_t now = time_us_32();
        
        // --- 1. Read IMU at ~1kHz ---
        if (now - last_sensor_read >= 1000) {
            last_sensor_read = now;
            
            // Read IMU if data ready
            if (ism330_data_ready(spi1, PIN_CS_IMU)) {
                uint8_t imu_buf[12];
                uint8_t cmd = 0x22 | 0x80;  // OUTX_L_G
                gpio_put(PIN_CS_IMU, 0);
                spi_write_blocking(spi1, &cmd, 1);
                spi_read_blocking(spi1, 0x00, imu_buf, 12);
                gpio_put(PIN_CS_IMU, 1);
                
                // Pack into tx_buffer (MSB-aligned)
                for (int i = 0; i < 12; i++) {
                    tx_buffer[4 + i] = ((uint32_t)imu_buf[i]) << 24;
                }
            }
        }
        
        // --- 2. Trigger Mag measurement every 10ms (100Hz) ---
        if (now - last_mag_trigger >= 10000) {
            last_mag_trigger = now;
            mmc5983_trigger_measurement(spi1, PIN_CS_MAG);
        }
        
        // --- 3. Read Mag if ready ---
        if (mmc5983_data_ready(spi1, PIN_CS_MAG)) {
            uint8_t mag_buf[7];  // 6 data + 1 status
            uint8_t cmd = 0x00 | 0x80;  // XOUT0
            gpio_put(PIN_CS_MAG, 0);
            spi_write_blocking(spi1, &cmd, 1);
            spi_read_blocking(spi1, 0x00, mag_buf, 7);
            gpio_put(PIN_CS_MAG, 1);
            
            // Reconstruct 16-bit values from 18-bit data (drop 2 LSBs for simplicity)
            // X: [17:10] from buf[0], [9:2] from buf[1]
            uint16_t mx = (((uint16_t)mag_buf[0]) << 8) | mag_buf[1];
            uint16_t my = (((uint16_t)mag_buf[2]) << 8) | mag_buf[3];
            uint16_t mz = (((uint16_t)mag_buf[4]) << 8) | mag_buf[5];
            
            // Pack into tx_buffer as 2 bytes each (6 bytes total)
            tx_buffer[16] = ((uint32_t)(mx >> 8)) << 24;
            tx_buffer[17] = ((uint32_t)(mx & 0xFF)) << 24;
            tx_buffer[18] = ((uint32_t)(my >> 8)) << 24;
            tx_buffer[19] = ((uint32_t)(my & 0xFF)) << 24;
            tx_buffer[20] = ((uint32_t)(mz >> 8)) << 24;
            tx_buffer[21] = ((uint32_t)(mz & 0xFF)) << 24;
        }

        // --- 4. Heartbeat ---
        static uint32_t last_heartbeat = 0;
        if (now - last_heartbeat > 1000000) {
            static uint32_t last_irq_count = 0;
            uint32_t current_irq = irq_count;
            printf("[MAIN] HB: CS IRQs: %d (D: %d), Rate: ~%d Hz\n", 
                   current_irq, current_irq - last_irq_count, current_irq - last_irq_count);
            last_irq_count = current_irq;
            last_heartbeat = now;
        }

        // --- 5. Re-arm CS IRQ ---
        if (now - last_rearm > 500) { 
            if (gpio_get(PIN_CS_SLAVE)) { 
                gpio_set_irq_enabled(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, true);
                last_rearm = now;
            }
        }
        sleep_us(50);
    }
}
