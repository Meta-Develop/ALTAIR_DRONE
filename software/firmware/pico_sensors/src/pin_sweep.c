#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Pin Sweep Diagnostic
// Scans all GPIOs (0-28) for a LOW signal (Active CS)
// Prints the detected pin to USB Serial

#define PIN_LED 25

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB
    printf("=== ALTAIR Pin Sweep Diagnostic ===\n");
    printf("Waiting for CS (Low Signal) on any pin...\n");

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Init all pins as Input Pull-Up
    for (int i = 0; i <= 28; i++) {
        if (i == PIN_LED) continue; // Skip LED
        if (i == 23 || i == 24) continue; // Skip SMPS pins on Pico 1? Pico 2 uses different. 
        // Just skip crucial internal ones if any. 
        // Pico 2 (RP2350): 0-29 usually available.
        // Let's safe-guard against USB pins if they are shared? 
        // USB is not GPIO.
        
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_pull_up(i); // CS is active Low, so Idle is High (Pull-Up)
    }

    uint32_t last_heartbeat = 0;

    while (true) {
        // Heartbeat (1Hz)
        uint32_t now = time_us_32();
        if (now - last_heartbeat > 500000) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_heartbeat = now;
        }

        // Scan all pins
        for (int i = 0; i <= 28; i++) {
            if (i == PIN_LED) continue;
            
            // If Pin is Low (CS Detected)
            if (!gpio_get(i)) {
                printf("Activity detected on PIN %d (Logic Low)\n", i);
                
                // Fast blink to confirm visual
                gpio_put(PIN_LED, 1);
                sleep_ms(10);
                gpio_put(PIN_LED, 0);
            }
        }
        
        sleep_ms(1); // valid scan rate
    }
}
