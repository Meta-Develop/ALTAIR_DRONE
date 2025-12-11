#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
// FreeRTOS includes REMOVED
// #include "FreeRTOS.h"
// #include "task.h"
// #include "semphr.h"

#include <uxr/client/transport.h>
#include "pico_uart_transports.h"
#include "tusb.h" // TinyUSB

// Mutex removed for Superloop (Single Threaded)

bool pico_serial_transport_open(struct uxrCustomTransport * transport){
    (void) transport;
    // No mutex needed
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void) transport;
    (void) err;

    // 1. Connection Check
    if (!tud_cdc_connected()) {
        return len; // Fake success to keep alive
    }

    uint64_t start_time = time_us_64();
    size_t total_written = 0;
    const uint64_t TIMEOUT_US = 10000; // 10ms hard timeout

    while (total_written < len) {
        // Direct write
        uint32_t av = tud_cdc_write_available();
        if (av > 0) {
            uint32_t to_write = (len - total_written);
            if (to_write > av) to_write = av;
            
            uint32_t written = tud_cdc_write(&buf[total_written], to_write);
            if (written > 0) {
                tud_cdc_write_flush(); 
                total_written += written;
            }
        } else {
             // Buffer full
             sleep_us(100); 
        }

        // 3. Timeout Check
        if ((time_us_64() - start_time) > TIMEOUT_US) {
            break; 
        }
    }
    
    // Always return len to keep micro-ROS alive
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    (void) transport;
    (void) err;
    
    uint64_t start_time = time_us_64();
    size_t read_count = 0;
    
    while (read_count < len && (time_us_64() - start_time) < (uint64_t)(timeout * 1000)) {
        if (tud_cdc_available()) {
             int n = tud_cdc_read(&buf[read_count], 1);
             if (n > 0) read_count += n;
        } else {
            sleep_us(100);
        }
    }
    return read_count;
}
