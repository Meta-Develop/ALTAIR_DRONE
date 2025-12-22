/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "spi_slave.pio.h"
#include "drivers/ism330dhcx.h" 

// --- CONFIGURATION ---
#define TOTAL_SIZE  36       // 4 header + 32 payload
#define PIN_MISO    19       // GP19 (Pin 25)
#define PIN_CS      17       // GP17 (Pin 22)
#define PIN_SCK     18       // GP18 (Pin 24)
#define PIN_LED     25       // Onboard LED

// Globals
PIO pio = pio0;
uint sm = 0;
uint offset = 0;
int dma_tx;
uint8_t tx_buffer[TOTAL_SIZE] __attribute__((aligned(4))); 
volatile uint32_t irq_count = 0;

// CS Falling Edge Interrupt Handler
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_CS && (events & GPIO_IRQ_EDGE_FALL)) {
        // 1. Disable IRQ 
        gpio_set_irq_enabled(PIN_CS, GPIO_IRQ_EDGE_FALL, false);
        
        // Debug
        gpio_xor_mask(1u << PIN_LED);
        irq_count++;
        
        // 2. Restart PIO and DMA
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(dma_tx);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset + spi_slave_offset_entry_point));
        
        // 3. Pre-fill Header (ALL ONES for Debug persistence)
        pio_sm_put(pio, sm, 0xFF); 
        pio_sm_put(pio, sm, 0xFF); 
        pio_sm_put(pio, sm, 0xFF); 
        pio_sm_put(pio, sm, 0xFF); 
        
        // 4. DMA the rest (DISABLED FOR DEBUG)
        // dma_channel_set_trans_count(dma_tx, TOTAL_SIZE - 4, false);
        // dma_channel_set_read_addr(dma_tx, tx_buffer + 4, true);
        
        // 5. Enable PIO (Wait for SCK)
        busy_wait_us_32(2); 
        pio_sm_set_enabled(pio, sm, true);
    }
}

int main() {
    stdio_init_all();
    
    // LED Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // --- CRITICAL: Explicitly Release MISO from any previous GPIO func ---
    gpio_init(PIN_MISO); 
    
    // Buffer Init
    tx_buffer[0] = 0xAA; tx_buffer[1] = 0xBB; tx_buffer[2] = 0xCC; tx_buffer[3] = 0xDD;
    for (int i = 4; i < TOTAL_SIZE; i++) tx_buffer[i] = (uint8_t)(i - 3);

    printf("Pico SPI Slave DMA (Fixed)\n");

    // PIO Init
    offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_init(pio, sm, offset, PIN_MISO);
    
    // --- CRITICAL: FORCE PIO PIN DIRECTION AGAIN ---
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_MISO, 1, true); 

    // DMA Init
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true)); 

    dma_channel_configure(dma_tx, &c, &pio->txf[sm], tx_buffer + 4, 0, false);
    
    // CS IRQ Init
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_up(PIN_CS); 
    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL, true, &cs_irq_handler);
    printf("[MAIN] CS IRQ Enabled.\n");
    
    uint32_t last_rearm = 0;
    
    while (true) {
        // Heartbeat
        static uint32_t last_heartbeat = 0;
        if (time_us_32() - last_heartbeat > 1000000) {
            static uint32_t last_irq_count = 0;
            uint32_t current_irq = irq_count;
            printf("[MAIN] Heartbeat. Uptime: %d ms, CS IRQ Count: %d (Delta: %d)\n", 
                   to_ms_since_boot(get_absolute_time()), current_irq, current_irq - last_irq_count);
            last_irq_count = current_irq;
            last_heartbeat = time_us_32();
        }

        // Debounced Re-arm
        if (time_us_32() - last_rearm > 500) { 
            if (gpio_get(PIN_CS)) { 
                gpio_set_irq_enabled(PIN_CS, GPIO_IRQ_EDGE_FALL, true);
                last_rearm = time_us_32();
            }
        }
        sleep_us(100);
    }
}
