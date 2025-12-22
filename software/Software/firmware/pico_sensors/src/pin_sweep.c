#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// CS Monitor - Real-time Logic Probe
// Monitors GP17 (CS) and GP18 (SCK)
// Prints state to USB Serial

#define PIN_CS 17
#define PIN_SCK 18
#define PIN_LED 25

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("=== ALTAIR CS Monitor ===\n");
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Inputs with weak Pull-Ups (Default for unconnected)
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_IN); gpio_pull_up(PIN_CS);
    
    gpio_init(PIN_SCK); gpio_set_dir(PIN_SCK, GPIO_IN); gpio_pull_down(PIN_SCK); // SCK usually Idle Low

    while (true) {
        int cs_val = gpio_get(PIN_CS);
        int sck_val = gpio_get(PIN_SCK);
        
        printf("CS(GP17): %d  |  SCK(GP18): %d\n", cs_val, sck_val);
        
        // Visual indicator on LED: ON if CS is Low (Active)
        gpio_put(PIN_LED, !cs_val);
        
        sleep_ms(100);
    }
}
