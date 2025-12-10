#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // For Mutex

#include <uxr/client/transport.h>
#include "pico_uart_transports.h"
#include "tusb.h" // TinyUSB

// Static mutex to protect TinyUSB access (Thread Safety Fix)
static SemaphoreHandle_t g_usb_mutex = NULL;

bool pico_serial_transport_open(struct uxrCustomTransport * transport){
    (void) transport;
    
    // Initialize Mutex if not already
    if (g_usb_mutex == NULL) {
        g_usb_mutex = xSemaphoreCreateMutex();
    }
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
        return len;
    }

    // Safety check for mutex
    if (g_usb_mutex == NULL) {
        g_usb_mutex = xSemaphoreCreateMutex();
    }

    uint64_t start_time = time_us_64();
    size_t total_written = 0;
    const uint64_t TIMEOUT_US = 10000; // 10ms hard timeout

    while (total_written < len) {
        // Protect TinyUSB call with Mutex
        if (xSemaphoreTake(g_usb_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            // Check available space
            uint32_t avail = tud_cdc_write_available();
            if (avail > 0) {
                uint32_t to_write = len - total_written;
                if (to_write > avail) to_write = avail;

                uint32_t written = tud_cdc_write(&buf[total_written], to_write);
                tud_cdc_write_flush(); 
                total_written += written;
            }
            
            xSemaphoreGive(g_usb_mutex);
        } else {
            // Failed to take mutex (contention)
            // Just yield and try again
        }

        // Buffer full or Mutex contention: Yield to OS
        if (total_written < len) {
             vTaskDelay(1);
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
        // Read doesn't need strict mutex usually if single consumer, 
        // but for safety we can add it or just rely on tud_cdc_available
        // which is generally safe for single consumer.
        if (tud_cdc_available()) {
             int n = tud_cdc_read(&buf[read_count], 1);
             if (n > 0) read_count += n;
        } else {
            vTaskDelay(1);
        }
    }
    return read_count;
}
