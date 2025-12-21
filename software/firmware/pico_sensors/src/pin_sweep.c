#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Test all SPI pins to help identify wiring mistakes
#define PIN_RX   16 
#define PIN_CS   17 
#define PIN_SCK  18 
#define PIN_TX   19 // MISO

#define PIN_LED 25

int main() {
    stdio_init_all();
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    gpio_init(PIN_RX);  gpio_set_dir(PIN_RX, GPIO_OUT);
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_init(PIN_SCK); gpio_set_dir(PIN_SCK, GPIO_OUT);
    gpio_init(PIN_TX);  gpio_set_dir(PIN_TX, GPIO_OUT);
    
    while (true) {
        // pattern: Blink LED fast to show start of sequence
        for(int i=0; i<3; i++) {
            gpio_put(PIN_LED, 1); sleep_ms(50);
            gpio_put(PIN_LED, 0); sleep_ms(50);
        }
        sleep_ms(500);

        // 1. Toggle GP16 (RX)
        // 2. Toggle GP17 (CS)
        // 3. Toggle GP18 (SCK)
        // 4. Toggle GP19 (TX/MISO)
        
        // We will toggle ALL of them together at 5Hz
        // If the user has connected to ANY of these, RPi should see it.
        
        for(int i=0; i<50; i++) { // 5 seconds of toggling
            bool state = (i % 2 == 0);
            gpio_put(PIN_RX, state);
            gpio_put(PIN_CS, state);
            gpio_put(PIN_SCK, state);
            gpio_put(PIN_TX, state);
            gpio_put(PIN_LED, state);
            sleep_ms(100);
        }
    }
}
