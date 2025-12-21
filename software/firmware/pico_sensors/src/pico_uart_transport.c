#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico_uart_transport.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We use the pins connected to the Debug Probe
// Probe GP4 (TX) -> Pico GP1 (RX)
// Probe GP5 (RX) -> Pico GP0 (TX)
#define UART_TX_PIN 0
#define UART_RX_PIN 1

bool pico_uart_transport_open(struct uxrCustomTransport * transport){
    // UART Init
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Optional: Flow control? No.
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true); // Enable FIFO
    
    return true;
}

bool pico_uart_transport_close(struct uxrCustomTransport * transport){
    uart_deinit(UART_ID);
    return true;
}

size_t pico_uart_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err){
    // Blocking write
    uart_write_blocking(UART_ID, buf, len);
    return len;
}

size_t pico_uart_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err){
    // Timeout Read
    // UART doesn't have a clean timeout read in SDK blocking API.
    // We must poll `uart_is_readable_within_us`.
    
    size_t read_count = 0;
    absolute_time_t timeout_time = make_timeout_time_ms(timeout);
    
    while(read_count < len) {
        if (!uart_is_readable_within_us(UART_ID, 1000)) { // 1ms poll
             if (absolute_time_diff_us(get_absolute_time(), timeout_time) < 0) {
                 break; // Timeout
             }
             continue;
        }
        
        // Read available
        buf[read_count++] = uart_getc(UART_ID);
    }
    
    return read_count;
}
