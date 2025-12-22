#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Test pins matching SPI layout
#define PIN_MISO 19  // TX to master - this should output
#define PIN_LED 25

int main() {
    stdio_init_all();
    
    // LED for status
    gpio_init(PIN_LED); 
    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Set MISO as GPIO output - toggle it to verify connectivity
    gpio_init(PIN_MISO);
    gpio_set_dir(PIN_MISO, GPIO_OUT);
    
    uint32_t count = 0;
    bool state = false;
    
    while (true) {
        // Toggle MISO pin at ~10Hz
        state = !state;
        gpio_put(PIN_MISO, state);
        gpio_put(PIN_LED, state);  // LED mirrors MISO
        
        sleep_ms(50);  // 10Hz toggle
        
        count++;
    }
}
