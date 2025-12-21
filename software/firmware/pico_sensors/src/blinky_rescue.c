/**
 * @file blinky_rescue.c
 * @brief Minimal Rescue Firmware - Blinks LED at 5Hz
 * used to identify which Pico is alive and verify flashing works.
 */

#include "pico/stdlib.h"

int main() {
    // 1. Skip Standard IO (Prevent Hangs)
    // stdio_init_all();

    // 2. Initialize LED Pin (GPIO 25 is standard Pico LED)
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // 3. Fast Blink Loop (5Hz) - Distinctive "Stress" Pattern
    // 100ms ON, 100ms OFF
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}
