/**
 * @file pico_spi_diag.c
 * @brief Diagnostic firmware to check CS Interrupts
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

volatile int irq_count = 0;
volatile bool cs_low = false;

void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_CS) {
        irq_count++;
        // Check physical level
        cs_low = !gpio_get(PIN_CS);
        // Visual Debug: Toggle LED on CS activity
        gpio_xor_mask(1 << 25);
    }
}

int main() {
    stdio_init_all();

    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // SPI Init
    spi_init(SPI_PORT, 100 * 1000); 
    spi_set_slave(SPI_PORT, true);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_up(PIN_CS); // Pull up to avoid floating trigger
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Interrupt - REMOVED for Polling Test
    // gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &cs_irq_handler);

    printf("Pico SPI Diag Polling Started.\n");
    
    // Quick blink to show boot
    for(int i=0; i<5; i++) {
        gpio_put(25, 1); sleep_ms(100);
        gpio_put(25, 0); sleep_ms(100);
    }

    while (true) {
        // Polling Mode:
        // Reads CS pin. If LOW (Active), Turn LED ON.
        // If HIGH (Inactive), Turn LED OFF.
        
        bool active = !gpio_get(PIN_CS); // Active Low
        gpio_put(25, active);
        
        // No sleep to maximize responsiveness
    }
}
