#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/stdio/driver.h"
#include "pico_uart_transports.h"

// Check if generic transport is available in uxr
// The signatures match uxr_custom_transport_open_t etc.

bool pico_serial_transport_open(struct uxrCustomTransport * transport){
    (void) transport;
    // Ensure strict binary mode (no CR/LF translation)
    stdio_set_translate_crlf(&stdio_usb, false);
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void) transport;
    (void) err;
    // Use stdio_usb specific raw write or standard fwrite?
    // stdio_usb_out_chars is non-blocking if not full?
    // We want blocking or reasonably reliable.
    // _write in newlib often maps to stdio.
    // Let's use fwrite to stdout which Pico SDK maps to USB if enabled.
    
    // However, stdio might modify CR/LF. We want raw binary.
    // In Pico SDK, stdio_usb defaults to translating \n to \r\n.
    // We MUST disable this for micro-ROS binary protocol.
    // This is usually done via `pico_enable_stdio_usb(..., 1)` in CMake? No.
    // We can use `stdio_set_translate_crlf(&stdio_usb, false)` if exposed.
    // Or we just use the lower level `stdio_usb_out_chars`.
    
    // Better: use `stdio_set_translate_crlf(&stdio_usb, false)` if possible, 
    // but `stdio_usb` struct might not be directly exposed easily.
    // Alternative: `stdio_usb_init()` sets it up.
    // Best practice for micro-ROS on Pico is to use `pico_stdio_usb` and ensure CRLF is off.
    
    // Let's assume `stdio_init_all()` or similar was called. 
    // For binary safety:
    
    // It's simpler to use `stdio_put_string` or `fwrite` IF raw.
    // But safely, let's try to find the low level SDK function.
    // `#include "pico/stdio/driver.h"`
    
    // For this task, I will use standard fwrite and assume CRLF is disabled or handled.
    // Actually, `stdio_usb_init` does not enable translation by default? 
    // Wait, `stdio_init_all` enables translation.
    
    // To be safe, I will use `stdio_set_translate_crlf` for all drivers.
    stdio_set_translate_crlf(&stdio_usb, false);
    
    size_t i = 0;
    uint64_t start_time = time_us_64();
    
    while(i < len) {
        // Timeout check (e.g. 100ms) to prevent hard lockup
        if ((time_us_64() - start_time) > 100000) {
            break;
        }

        int written = putchar_raw(buf[i]);
        if (written != PICO_ERROR_GENERIC) {
            i++;
            // Reset timeout on successful write
            start_time = time_us_64();
        } else {
            // Buffer full? Yield slightly
            sleep_us(10);
        }
    }
    return i;
}

size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    (void) transport;
    (void) err;
    
    uint64_t start_time = time_us_64();
    size_t read_count = 0;
    
    while (read_count < len && (time_us_64() - start_time) < (uint64_t)(timeout * 1000)) {
        int c = getchar_timeout_us(0); // Non-blocking check
        if (c != PICO_ERROR_TIMEOUT) {
            buf[read_count++] = (uint8_t)c;
        } else {
            // Small delay to yield?
            sleep_us(10);
        }
    }
    return read_count;
}
