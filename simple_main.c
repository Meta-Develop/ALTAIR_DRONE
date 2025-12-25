#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Use SPI0 as Slave
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

void simple_delay() {
    volatile int i = 0;
    for (i = 0; i < 1000000; i++) {
        __asm volatile ("nop");
    }
}

int main() {
    // stdio_init_all(); 
    
    // SPI Setup
    spi_init(SPI_PORT, 1000000); // 1MHz
    spi_set_slave(SPI_PORT, true);
    
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Define simple test pattern
    uint8_t test_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};

    while(true) {
        if (spi_is_writable(SPI_PORT)) {
            spi_write_blocking(SPI_PORT, test_data, 4);
        }
        
        // simple_delay(); 
    }
}
