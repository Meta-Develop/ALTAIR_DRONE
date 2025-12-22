#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Test CS Input
#define PIN_CS   17 
#define PIN_LED  25

int main() {
    stdio_init_all();
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // CS is Input
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    // gpio_pull_up(PIN_CS); // Optional: Default is usually no pull or pull-up
    
    while (true) {
        // Reflect CS state on LED
        // If CS is High (Inactive), LED ON
        // If CS is Low (Active), LED OFF
        if (gpio_get(PIN_CS)) {
            gpio_put(PIN_LED, 1);
        } else {
            gpio_put(PIN_LED, 0);
        }
    }
}
