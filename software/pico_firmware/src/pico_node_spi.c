/**
 * @file pico_node_spi.c
 * @brief DMA-based SPI Slave Firmware
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "drivers/mpu6050.h"

// --- Config ---
#define SPI_PORT spi0
#define PIN_MISO 16 // RX in Slave Mode (MOSI) -> but wait. MISO is Output. GP16 is RX. GP19 is TX.
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Wiring:
// RPi MOSI(19) -> Pico GP16 (RX)
// RPi MISO(21) -> Pico GP19 (TX)
// RPi SCK(23)  -> Pico GP18 (SCK)
// RPi CS(24)   -> Pico GP17 (CSn)

#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

// Payload: 8 floats = 32 bytes
#define PAYLOAD_FLOATS 8
#define PAYLOAD_BYTES (PAYLOAD_FLOATS * sizeof(float))

// --- Globals ---
// Use 2 buffers? Or just one? 
// Single buffer is fine if main loop updates it atomically-ish.
float packet_data[PAYLOAD_FLOATS]; // Source
uint8_t tx_buffer[PAYLOAD_BYTES];  // DMA Source (Byte aligned)

int dma_tx;
dma_channel_config dma_config;

// --- ISR ---
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_CS) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Transaction Start
            // Visualization only
            gpio_xor_mask(1 << 25);
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Transaction End
            // 1. Abort current transfer (if any left)
            dma_channel_abort(dma_tx);
            
            // 2. Clear any garbage in RX FIFO (we don't use it but good practice)
            while (spi_is_readable(SPI_PORT)) {
                (void)spi_get_hw(SPI_PORT)->dr;
            }
            
            // 3. Clear TX FIFO (Drain overflow/stale)
            // HW_SPI_DR is write-only for TX, can't 'clear'.
            // But we can reset the peripheral? No, too slow.
            // Actually, if DMA filled it and Master read it, it should be empty.
            // If Master aborted early, it might be full.
            // We just proceed. New data will append.
            // (Ideally, we'd FLUSH TX, but RP2040 SPI has no flush bit other than disable/enable)
            
            // 4. Restart DMA for NEXT transaction immediately!
            // This fills the FIFO (8 bytes) and waits for DREQ.
            dma_channel_set_read_addr(dma_tx, tx_buffer, false);
            dma_channel_set_trans_count(dma_tx, PAYLOAD_BYTES, false);
            dma_channel_start(dma_tx);
        }
    }
}

int main() {
    stdio_init_all();

    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // I2C Init
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    mpu6050_init(I2C_PORT);

    // SPI Init (Slave)
    spi_init(SPI_PORT, 4000 * 1000); // 4MHz matching Master
    spi_set_slave(SPI_PORT, true);
    
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI); // GP16 (Pico RX / MOSI)
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI); // GP17 (CSn)
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI); // GP18 (SCK)
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI); // GP19 (Pico TX / MISO)
    
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // DMA Setup
    dma_tx = dma_claim_unused_channel(true);
    dma_config = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8); // Byte transfers
    channel_config_set_dreq(&dma_config, spi_get_dreq(SPI_PORT, true)); // TX DREQ
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false); // Write to same address (SPI DR)
    
    dma_channel_configure(
        dma_tx,
        &dma_config,
        &spi_get_hw(SPI_PORT)->dr, // Dest: SPI Data Register
        tx_buffer,                 // Source: Buffer
        PAYLOAD_BYTES,             // Count
        false                      // Don't start yet
    );

    // CS Interrupt
    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &cs_irq_handler);

    // Initialize buffer with something visible (debug)
    for (int i=0; i<PAYLOAD_BYTES; i++) tx_buffer[i] = i; 

    // Prime the DMA pump!
    dma_channel_start(dma_tx);

    // Main Loop
    mpu6050_data_t imu_data;
    uint64_t last_time = time_us_64();

    // Initialize buffer with something visible (debug)
    for (int i=0; i<PAYLOAD_BYTES; i++) tx_buffer[i] = i; 

    while (true) {
        uint64_t now = time_us_64();
        if (now - last_time >= 1000) {
            last_time += 1000;

            // 1. Read IMU
            mpu6050_read_burst(I2C_PORT, &imu_data);

            // 2. Pack Data
            uint64_t t = time_us_64();
            packet_data[0] = (float)(t / 1000000);       
            packet_data[1] = (float)((t % 1000000) * 1000); 
            packet_data[2] = (float)imu_data.ax;
            packet_data[3] = (float)imu_data.ay;
            packet_data[4] = (float)imu_data.az;
            packet_data[5] = (float)imu_data.gx;
            packet_data[6] = (float)imu_data.gy;
            packet_data[7] = (float)imu_data.gz;

            // 3. Update DMA Buffer
            // FIXED PATTERN TEST
            // Fill with incrementing bytes 0..31 to verify byte order and link
            for (int i=0; i<PAYLOAD_BYTES; i++) {
                tx_buffer[i] = (uint8_t)(i + 10); // 10, 11, 12...
            }
            // Keep the LED logic to know the loop is running
            
            // Visual I2C Check:
            // Check if Az is roughly 1G (approx 16384 LSB for 2g range, or just > 1000)
            // If data is valid (non-zero), blink FAST.
            // If data is zero (I2C fail), blink SLOW.
            int16_t az = imu_data.az;
            static int led_timer = 0;
            static bool led_state = false;
            
            bool data_ok = (az > 2000 || az < -2000); 
            // Threshold 2000 covers 1g (16384) easily, but filters noise vs dead zero.
            
            int blink_interval = data_ok ? 100 : 1000; // 10Hz vs 1Hz (loop is 1ms)
            
            if (++led_timer >= blink_interval) {
                led_timer = 0;
                led_state = !led_state;
                gpio_put(25, led_state);
            }
        }
    }
}
