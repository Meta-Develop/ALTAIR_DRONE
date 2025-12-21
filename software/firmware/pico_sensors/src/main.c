#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h" 
#include "hardware/structs/io_bank0.h" // For raw pad control (io_bank0/pads_bank0)
#include "hardware/structs/pads_bank0.h"

// Safe Mode Firmware v2.1 (Input Sync Bypass + 1Hz Heartbeat)
// - 1Hz Heartbeat (Non-blocking)
// - Polling SPI Slave
// - INPUT SYNC BYPASS Enabled for SCK, MOSI, CS

#define SPI_PORT spi0
#define PIN_RX   16 // MOSI (Input)
#define PIN_CS   17 // CS (Input)
#define PIN_SCK  18 // SCK (Input)
#define PIN_TX   19 // MISO (Output)

#define PIN_LED 25

// Configure Input Sync Bypass for lower latency
void configure_fast_spi_slave() {
    // SCK (GPIO 18)
    hw_write_masked(
        &padsbank0_hw->io[PIN_SCK], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS
    );
    
    // MOSI (GPIO 16)
    hw_write_masked(
        &padsbank0_hw->io[PIN_RX], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS
    );

    // CS (GPIO 17)
    hw_write_masked(
        &padsbank0_hw->io[PIN_CS], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS
    );
}

int main() {
    stdio_init_all();
    gpio_init(PIN_LED); 
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // SPI Slave Init
    spi_init(SPI_PORT, 1000000); 
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,  GPIO_FUNC_SPI);

    // Enable Fast Input
    configure_fast_spi_slave();

    uint8_t payload[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    int p_idx = 0;

    // Pre-fill FIFO
    spi_get_hw(SPI_PORT)->dr = payload[0];

    uint32_t last_time = 0;

    while (true) {
        // Heartbeat (1Hz)
        uint32_t now = time_us_32();
        if (now - last_time > 500000) { 
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_time = now;
        }

        // Poll SPI
        if (spi_is_readable(SPI_PORT)) {
            uint8_t rx = (uint8_t)spi_get_hw(SPI_PORT)->dr; 
            (void)rx; 
            
            p_idx = (p_idx + 1) % 4;
            spi_get_hw(SPI_PORT)->dr = payload[p_idx]; 
        }
    }
}
