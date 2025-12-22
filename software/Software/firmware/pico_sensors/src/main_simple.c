#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// --- CONFIG ---
// Simple SPI slave without DMA for reliable debugging
// Payload: 96 bytes = 24 floats (matches spi_bridge.py)
#define PAYLOAD_BYTES 96

// --- PINS (Pico 2 A) ---
// Link (Slave) - SPI0
#define SPI_LINK_PORT spi0
#define PIN_LINK_RX   16 // MOSI (from master)
#define PIN_LINK_CS   17 // CSn
#define PIN_LINK_SCK  18 // SCK
#define PIN_LINK_TX   19 // MISO (to master)

// STATUS
#define PIN_LED 25

// --- Payload Buffer ---
static uint8_t tx_buffer[PAYLOAD_BYTES] __attribute__((aligned(4)));

int main() {
    stdio_init_all();
    
    // LED for status
    gpio_init(PIN_LED); 
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);  // LED ON at start
    
    // --- HARDWARE SPI0 SLAVE ---
    spi_init(SPI_LINK_PORT, 1000000);
    spi_set_slave(SPI_LINK_PORT, true);
    spi_set_format(SPI_LINK_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(PIN_LINK_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_LINK_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_LINK_TX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_LINK_CS,  GPIO_FUNC_SPI);
    
    // Prepare constant test buffer with header and dummy data
    memset(tx_buffer, 0, PAYLOAD_BYTES);
    
    // Header at bytes 0-3
    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0xBB;
    tx_buffer[2] = 0xCC;
    tx_buffer[3] = 0xDD;
    
    // IMU dummy data at float indices 2-7 (bytes 8-31)
    float imu[6] = {0.01f, 0.02f, 9.81f, 0.001f, 0.002f, 0.003f};
    memcpy(&tx_buffer[8], imu, sizeof(imu));
    
    // Mag dummy data at float indices 8-10 (bytes 32-43)
    float mag[3] = {25.0f, 5.0f, 45.0f};
    memcpy(&tx_buffer[32], mag, sizeof(mag));
    
    // Pre-fill TX FIFO with data
    for (int i = 0; i < PAYLOAD_BYTES && spi_is_writable(SPI_LINK_PORT); i++) {
        spi_get_hw(SPI_LINK_PORT)->dr = tx_buffer[i];
    }
    
    uint32_t loop_count = 0;
    
    while (true) {
        // Keep TX FIFO filled
        static uint8_t tx_idx = 0;
        
        while (spi_is_writable(SPI_LINK_PORT)) {
            spi_get_hw(SPI_LINK_PORT)->dr = tx_buffer[tx_idx];
            tx_idx = (tx_idx + 1) % PAYLOAD_BYTES;
        }
        
        // Drain RX FIFO (we don't need the data)
        while (spi_is_readable(SPI_LINK_PORT)) {
            volatile uint8_t dummy = spi_get_hw(SPI_LINK_PORT)->dr;
            (void)dummy;
        }
        
        // Blink LED slowly to show we're alive
        loop_count++;
        if (loop_count > 100000) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            loop_count = 0;
        }

    }
}
