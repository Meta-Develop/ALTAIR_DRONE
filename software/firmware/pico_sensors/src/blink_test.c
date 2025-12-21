#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define PIN_LED 25

int main() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    while (true) {
        gpio_put(PIN_LED, 1);
        sleep_ms(100);
        gpio_put(PIN_LED, 0);
        sleep_ms(100);
    }
}
