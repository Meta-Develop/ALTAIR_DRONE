#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico_node.h"

int main() {
    stdio_init_all();
    
    // LED Init
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Signal Boot
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);

    // Init Node
    pico_node_init();
    
    // Start Scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while (1);
    return 0;
}
