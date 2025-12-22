#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Test SCLK Input
#define PIN_SCK  18 
#define PIN_LED  25

int main() {
    stdio_init_all();
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // SCK is Input
    gpio_init(PIN_SCK);
    gpio_set_dir(PIN_SCK, GPIO_IN);
    
    while (true) {
        // Reflect SCK state on LED
        if (gpio_get(PIN_SCK)) {
            gpio_put(PIN_LED, 1);
        } else {
            gpio_put(PIN_LED, 0);
        }
    }
}
