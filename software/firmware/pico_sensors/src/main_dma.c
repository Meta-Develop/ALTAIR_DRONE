/**
 * SPI SLAVE FIRMWARE WITH ISM330DHCX SENSOR
 * - Reads Accel/Gyro from ISM330DHCX via SPI1 (Master)
 * - Transmits sensor data to RPi4 via PIO SPI Slave (on CS IRQ)
 * - Sends 16 bytes: 4 header (AA BB CC DD) + 12 sensor data (6 Gyro + 6 Accel raw bytes)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "spi_slave.pio.h"  // Generated PIO header
#include "drivers/ism330dhcx.h" 

// --- PIN CONFIGURATION ---
// SPI0 PIO Slave (to RPi4 Master)
#define PIN_MISO_SLAVE    19       // GP19 (Pico Pin 25) - Output to RPi4
#define PIN_CS_SLAVE      17       // GP17 (Pico Pin 22) - Input from RPi4
#define PIN_SCK_SLAVE     18       // GP18 (Pico Pin 24) - Input from RPi4
#define PIN_MOSI_SLAVE    16       // GP16 (Pico Pin 21) - Input from RPi4 (unused for slave TX)

// SPI1 Master (to ISM330DHCX Sensor)
#define PIN_MISO_SENSOR   12       // GP12 (Pico Pin 16) - Input from Sensor
#define PIN_MOSI_SENSOR   11       // GP11 (Pico Pin 15) - Output to Sensor
#define PIN_SCK_SENSOR    10       // GP10 (Pico Pin 14) - Clock to Sensor
#define PIN_CS_SENSOR     13       // GP13 (Pico Pin 17) - CS to Sensor

#define PIN_LED     25       // Onboard LED

// --- PACKET FORMAT ---
// Total 16 bytes:
// [0-3]: Header AA BB CC DD (4 bytes)
// [4-15]: Raw sensor data: GX(2), GY(2), GZ(2), AX(2), AY(2), AZ(2) = 12 bytes 
#define TOTAL_BYTES  16

// Globals
PIO pio = pio0;
uint sm = 0;
uint offset = 0;
int dma_tx;

// TX Buffer: MSB-aligned 32-bit words (1 byte per word for PIO Left Shift)
uint32_t tx_buffer[TOTAL_BYTES] __attribute__((aligned(4))); 
volatile uint32_t irq_count = 0;

// Sensor Data Buffer (Raw, filled by main loop)
volatile int16_t sensor_raw[6];  // [GX, GY, GZ, AX, AY, AZ]
volatile bool sensor_data_ready = false;

// Forward declaration
void update_tx_buffer_from_sensor();

// CS Falling Edge Interrupt Handler
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_CS_SLAVE && (events & GPIO_IRQ_EDGE_FALL)) {
        // 1. Disable IRQ 
        gpio_set_irq_enabled(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, false);
        
        // Debug
        gpio_xor_mask(1u << PIN_LED);
        irq_count++;
        
        // 2. Restart PIO and DMA
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(dma_tx);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset + spi_slave_offset_entry_point));
        
        // 3. Pre-fill Header (AA BB CC DD)
        pio_sm_put(pio, sm, 0xAA000000); 
        pio_sm_put(pio, sm, 0xBB000000); 
        pio_sm_put(pio, sm, 0xCC000000); 
        pio_sm_put(pio, sm, 0xDD000000); 
        
        // 4. DMA the sensor data portion (TOTAL_BYTES - 4 words)
        dma_channel_set_trans_count(dma_tx, TOTAL_BYTES - 4, false);
        dma_channel_set_read_addr(dma_tx, tx_buffer + 4, true);
        
        // 5. Enable PIO
        busy_wait_us_32(2); 
        pio_sm_set_enabled(pio, sm, true);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500);  // Wait for USB enumeration
    
    printf("=== Pico SPI Slave + ISM330DHCX ===\n");

    // LED Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // --- SPI1 MASTER INIT (for ISM330DHCX Sensor) ---
    spi_init(spi1, 8 * 1000 * 1000);  // 8MHz SPI Clock to sensor
    gpio_set_function(PIN_MISO_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_SENSOR, GPIO_FUNC_SPI);
    
    // Initialize ISM330DHCX
    printf("[MAIN] Initializing ISM330DHCX...\n");
    if (!ism330_init(spi1, PIN_CS_SENSOR)) {
        printf("[MAIN] ISM330DHCX INIT FAILED! Check wiring.\n");
        // Continue anyway for debugging - will send zeros
    }

    // --- PIO SPI SLAVE INIT (for RPi4 Master) ---
    gpio_init(PIN_MISO_SLAVE); 
    
    // Buffer Init (Header + placeholder zeros for sensor)
    tx_buffer[0] = 0xAA000000; 
    tx_buffer[1] = 0xBB000000; 
    tx_buffer[2] = 0xCC000000; 
    tx_buffer[3] = 0xDD000000;
    for (int i = 4; i < TOTAL_BYTES; i++) tx_buffer[i] = 0;  // Zero sensor area

    // PIO Init
    offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_init(pio, sm, offset, PIN_MISO_SLAVE);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_MISO_SLAVE, 1, true); 

    // Input Init for SCK/MOSI (from RPi4 Master)
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
    printf("[MAIN] CS IRQ Enabled. Waiting for RPi4 SPI transactions...\n");
    
    uint32_t last_rearm = 0;
    uint32_t last_sensor_read = 0;
    ism330_data_t imu_data;
    
    while (true) {
        // --- 1. Read Sensor at ~1kHz ---
        uint32_t now = time_us_32();
        if (now - last_sensor_read >= 1000) {  // 1000us = 1kHz
            last_sensor_read = now;
            
            // Check sensor data ready (optional, can just read continuously)
            if (ism330_data_ready(spi1, PIN_CS_SENSOR)) {
                // Burst read raw data directly
                uint8_t raw_buf[12];  // 6 bytes gyro + 6 bytes accel
                
                // Read from 0x22 (OUTX_L_G) for 12 bytes
                uint8_t cmd = 0x22 | 0x80;  // Read + auto-increment
                gpio_put(PIN_CS_SENSOR, 0);
                spi_write_blocking(spi1, &cmd, 1);
                spi_read_blocking(spi1, 0x00, raw_buf, 12);
                gpio_put(PIN_CS_SENSOR, 1);
                
                // Pack into tx_buffer (MSB-aligned 32-bit words)
                // Each raw byte becomes one 32-bit word with byte in MSB position
                for (int i = 0; i < 12; i++) {
                    tx_buffer[4 + i] = ((uint32_t)raw_buf[i]) << 24;
                }
            }
        }
        
        // --- 2. Heartbeat ---
        static uint32_t last_heartbeat = 0;
        if (now - last_heartbeat > 1000000) {
            static uint32_t last_irq_count = 0;
            uint32_t current_irq = irq_count;
            printf("[MAIN] Heartbeat. CS IRQs: %d (Delta: %d), SPI Rate: ~%d Hz\n", 
                   current_irq, current_irq - last_irq_count, current_irq - last_irq_count);
            last_irq_count = current_irq;
            last_heartbeat = now;
        }

        // --- 3. Debounced Re-arm CS IRQ ---
        if (now - last_rearm > 500) { 
            if (gpio_get(PIN_CS_SLAVE)) { 
                gpio_set_irq_enabled(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL, true);
                last_rearm = now;
            }
        }
        sleep_us(50);  // Yield
    }
}
