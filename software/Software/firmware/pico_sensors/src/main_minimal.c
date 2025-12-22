#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    
    // Wait for USB (Optional but good)
    // while (!stdio_usb_connected()) { sleep_ms(100); }

    while (true) {
        printf("TEST %llu\r\n", time_us_64());
        sleep_ms(1000);
    }
    return 0;
}
