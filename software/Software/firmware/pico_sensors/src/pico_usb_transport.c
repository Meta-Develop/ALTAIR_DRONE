#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

// Micro-ROS Transport Definitions for USB Serial (via Stdio)

bool pico_usb_transport_open(struct uxrCustomTransport * transport) {
    // USB is initialized by stdio_init_all() in main()
    (void) transport;
    return true;
}

bool pico_usb_transport_close(struct uxrCustomTransport * transport) {
    (void) transport;
    return true;
}

size_t pico_usb_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
    (void) transport;
    (void) err;
    // Write to USB Stdout
    // fwrite blocks until written? or puts?
    // standard output is buffered?
    // We use low level if possible, or just standard.
    // stdio_usb usually handles this.
    
    // Efficient block write:
    // There isn't a direct block write in pico_stdlib stdio without looking into internals.
    // Loop putchar is fine for now, or fwrite.
    
    for (size_t i = 0; i < len; i++) {
        putchar(buf[i]);
    }
    fflush(stdout); // FORCE FLUSH
    return len;
}

size_t pico_usb_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
    (void) transport;
    (void) err;
    
    // Read from Stdin with timeout
    // absolute_time_t timeout_time = make_timeout_time_ms(timeout); // timeout is int (ms?) typically
    
    int64_t timeout_us = timeout * 1000;
    
    size_t read_count = 0;
    while (read_count < len) {
        int c = getchar_timeout_us(timeout_us);
        if (c == PICO_ERROR_TIMEOUT) {
            break;
        }
        buf[read_count++] = (uint8_t)c;
        // Decrease timeout for subsequent chars? 
        // Or is timeout total?
        // Usually read returns what is available.
        // We will return what we got.
        if (read_count > 0 && !stdio_usb_connected()) break; 
        
        // For remaining chars, we shouldn't wait full timeout again.
        // Just peek? 
        timeout_us = 0; // Immediate check for rest of burst
    }
    return read_count;
}
