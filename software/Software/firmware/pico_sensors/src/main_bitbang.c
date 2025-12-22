#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// BIT-BANG SPI SLAVE DEBUG
// LED LOGIC:
// ON  = Idle (Waiting for CS)
// OFF = Active (CS Low)

#define PIN_RX   16 // MOSI
#define PIN_CS   17 // CS
#define PIN_SCK  18 // SCK
#define PIN_TX   19 // MISO
#define PIN_LED  25

int main() {
    stdio_init_all();
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Inputs
    gpio_init(PIN_RX); gpio_set_dir(PIN_RX, GPIO_IN);
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_init(PIN_SCK); gpio_set_dir(PIN_SCK, GPIO_IN);
    
    // Output
    gpio_init(PIN_TX); gpio_set_dir(PIN_TX, GPIO_OUT);
    gpio_put(PIN_TX, 0); 
    
    // Increase drive strength for MISO
    gpio_set_drive_strength(PIN_TX, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(PIN_TX, GPIO_SLEW_RATE_FAST);
    
    const uint8_t byte_to_send = 0xAA; // 10101010
    
    while (true) {
        // Wait for CS Low (Active)
        gpio_put(PIN_LED, 1); // LED ON = Waiting (Idle)
        
        while (gpio_get(PIN_CS)) {
             // Idle Loop
             gpio_put(PIN_TX, 0); 
        }
        
        gpio_put(PIN_LED, 0); // LED OFF = Active Transaction
        
        // CS is now Low. Transaction started.
        // We need to output the first bit immediately (MSB of 0xAA = 1)
        int bit_idx = 7;
        gpio_put(PIN_TX, (byte_to_send >> bit_idx) & 1);
        
        while (!gpio_get(PIN_CS)) { // While CS is Low
            // Wait for SCK Rise (Master Sample)
            while (gpio_get(PIN_SCK) == 0 && !gpio_get(PIN_CS)) asm volatile("nop");
            if (gpio_get(PIN_CS)) break; 
            
            // Now SCK is High. Master is sampling us.
            
            // Wait for SCK Fall (Next bit setup)
            while (gpio_get(PIN_SCK) == 1 && !gpio_get(PIN_CS)) asm volatile("nop");
            if (gpio_get(PIN_CS)) break;
            
            // SCK is Low. Setup next bit.
            bit_idx--;
            if (bit_idx < 0) bit_idx = 7; // Repeat byte
            gpio_put(PIN_TX, (byte_to_send >> bit_idx) & 1);
        }
    }
}
