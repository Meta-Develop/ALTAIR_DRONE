/**
 * Pico Sensors - Non-Blocking SPI approach
 * CRITICAL: Never block waiting for SPI - just write what fits
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// SPI SLAVE PINS (SPI0)
#define PIN_MISO_SLAVE  19
#define PIN_SCK_SLAVE   18
#define PIN_MOSI_SLAVE  16
#define PIN_CS_SLAVE    17

#define PIN_TRIGGER     22
#define PIN_LED         25

#define PACKET_SIZE 36

// Test buffer
uint8_t tx_buffer[PACKET_SIZE];

int main() {
    stdio_init_all();
    sleep_ms(100);

    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Trigger Pin
    gpio_init(PIN_TRIGGER);
    gpio_set_dir(PIN_TRIGGER, GPIO_OUT);
    gpio_put(PIN_TRIGGER, 0);
    
    // SPI Setup (Slave mode)
    spi_init(spi0, 10000000);
    spi_set_slave(spi0, true);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(PIN_MISO_SLAVE, GPIO_FUNC_SPI);
    gpio_set_drive_strength(PIN_MISO_SLAVE, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(PIN_MISO_SLAVE, GPIO_SLEW_RATE_FAST);
    gpio_set_function(PIN_SCK_SLAVE, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_SLAVE, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS_SLAVE, GPIO_FUNC_SPI); 
    gpio_pull_up(PIN_CS_SLAVE);

    // Fill test buffer with 0xAA55 pattern
    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0x55;
    for(int i=2; i<PACKET_SIZE; i++) {
        tx_buffer[i] = i;
    }

    printf("Non-Blocking SPI Test Running...\n");

    uint32_t count = 0;
    
    while(1) {
        // NON-BLOCKING approach:
        // 1. Pre-fill FIFO with as much as possible (8 bytes max FIFO depth)
        int filled = 0;
        for(int i=0; i<8 && i<PACKET_SIZE; i++) {
            if (spi_is_writable(spi0)) {
                spi_get_hw(spi0)->dr = tx_buffer[i];
                filled++;
            }
        }
        
        // 2. Trigger HIGH (RPi starts clocking)
        gpio_put(PIN_TRIGGER, 1);
        
        // 3. Continue filling as RPi clocks (non-blocking, with timeout)
        uint64_t start = time_us_64();
        int tx_idx = filled;
        
        while(tx_idx < PACKET_SIZE && (time_us_64() - start < 200)) {
            if (spi_is_writable(spi0)) {
                spi_get_hw(spi0)->dr = tx_buffer[tx_idx++];
            }
        }
        
        // 4. Keep trigger HIGH for 50us minimum
        while(time_us_64() - start < 50);
        gpio_put(PIN_TRIGGER, 0);
        
        // 5. Wait remainder of 166us period
        while(time_us_64() - start < 166);
        
        count++;
        if (count % 6000 == 0) {
            gpio_xor_mask(1u << PIN_LED);
        }
    }
}
