/**
 * MISO (GP19) HARDWARE TOGGLE TEST
 * 
 * This firmware purely toggles GP19 (Pin 25) and the LED (GP25).
 * USED TO VERIFY IF THE PIN DRIVER IS WORKING.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define PIN_LED  25
#define PIN_MISO 19 // GP19 (Pin 25)

int main() {
    stdio_init_all();
    
    // Setup LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Setup MISO (GP19) as simple GPIO Output
    gpio_init(PIN_MISO);
    gpio_set_dir(PIN_MISO, GPIO_OUT);
    
    printf("--- MISO TOGGLE TEST STARTING ---\n");
    printf("Toggling GP19 (Pin 25) and LED (Pin TP5) at 5Hz\n");

    while (true) {
        // High
        gpio_put(PIN_LED, 1);
        gpio_put(PIN_MISO, 1);
        sleep_ms(100);
        
        // Low
        gpio_put(PIN_LED, 0);
        gpio_put(PIN_MISO, 0);
        sleep_ms(100);
        // printf("Toggle...\n"); // Commented out to reduce noise
    }
}
