#ifndef PICO_UART_TRANSPORTS_H
#define PICO_UART_TRANSPORTS_H

#include <stdbool.h>
#include "pico/stdlib.h"
#include <uxr/client/transport.h>

bool pico_serial_transport_open(struct uxrCustomTransport * transport);
bool pico_serial_transport_close(struct uxrCustomTransport * transport);
size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#endif // PICO_UART_TRANSPORTS_H
