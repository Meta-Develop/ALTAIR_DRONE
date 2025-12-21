#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Hardware Connectivity Test
// MISO (GP19) -> HIGH
// DATA READY (GP20) -> HIGH (To trigger bridge)

#define PIN_MISO 19
#define PIN_DATA_READY 20
#define PIN_LED 25

int main() {
    stdio_init_all();
    
    gpio_init(PIN_LED); 
    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Drive Handshake High to ensure Bridge runs
    gpio_init(PIN_DATA_READY); 
    gpio_set_dir(PIN_DATA_READY, GPIO_OUT);
    gpio_put(PIN_DATA_READY, 1);

    // Drive MISO High to test wire
    gpio_init(PIN_MISO);
    gpio_set_dir(PIN_MISO, GPIO_OUT);
    gpio_put(PIN_MISO, 1);

    while (true) {
        // Blink LED to show life
        gpio_put(PIN_LED, 1);
        sleep_ms(200);
        gpio_put(PIN_LED, 0);
        sleep_ms(200);
        
        // Reinforce outputs
        gpio_put(PIN_MISO, 1);
        gpio_put(PIN_DATA_READY, 1);
    }
}
