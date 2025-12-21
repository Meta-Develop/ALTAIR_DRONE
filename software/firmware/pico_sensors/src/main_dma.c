#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "spi_slave.pio.h" // PIO Header

// Sensor Driver
#include "drivers/ism330dhcx.h"

// =============================================================================
// DMA-Based SPI Slave Firmware with Real Sensor Reading
// Target: RP2350 (Pico 2) - Pico 2A (Sensors)
// 
// Architecture:
//   - PIO SPI Slave: Responds to RPi4 master (GP16-19)
//   - SPI1 Master: Reads ISM330DHCX sensor (GP12-15)
//   - Timer: 1000Hz interrupt to read sensor
//   - DMA: Automatically fills PIO TX FIFO from buffer
// =============================================================================

// --- PIO SPI Slave (Link to RPi4) ---
#define PIN_MISO 19
#define PIN_MOSI 16
#define PIN_SCK  18
#define PIN_CS   17

// --- SPI1 Master (Sensor) ---
#define SPI_SENSOR_PORT spi1
#define PIN_SENSOR_SCK  14  // GP14 -> Sensor SCL
#define PIN_SENSOR_TX   15  // GP15 -> Sensor SDA (MOSI)
#define PIN_SENSOR_RX   12  // GP12 -> Sensor SDO (MISO)
#define PIN_SENSOR_CS   13  // GP13 -> ISM330 ACS

// --- Handshake & Status ---
#define PIN_DATA_READY 20  // Pin 26 -> RPi GPIO 6 (Pin 31)
#define PIN_CS_LOOPBACK 21 // Pin 27 <- Jumper from Pin 22 (GP17 CS) for edge detection
#define PIN_LED 25

// --- Payload ---
// 96 bytes = 24 floats
// Layout: [Header 4B][Reserved 4B][Accel 12B][Gyro 12B][Mag 12B][Baro 8B][Range 4B][Padding]
#define PAYLOAD_SIZE 96
#define TOTAL_SIZE (PAYLOAD_SIZE + 4)

// Header bytes
static const uint8_t HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

// Global buffer (aligned for DMA)
static uint8_t tx_buffer[TOTAL_SIZE] __attribute__((aligned(4)));

// Sensor data (volatile for ISR access)
static volatile ism330_data_t sensor_data;
static volatile bool new_data_ready = false;
static volatile uint32_t sample_count = 0;

// DMA
int dma_tx;
PIO pio = pio0;
uint sm = 0;

// CS Edge Callback - Resync DMA
// PIO IRQ Handler (Replaces CS Loopback)
void pio_irq_handler() {
    // Check if it's our SM causing IRQ
    if (pio0_hw->irq & 1) {
        pio0_hw->irq = 1; // Clear IRQ 0
        
        // Debug: Toggle LED to prove IRQ fired (CS Detected)
        gpio_xor_mask(1u << PIN_LED);

        // Transaction Start (CS Low detected by PIO)
        
        // 1. Abort current DMA to reset pointer
        dma_channel_abort(dma_tx);
        
        // 2. Clear FIFOs if needed (optional)
        // pio_sm_clear_fifos(pio, sm);
        
        // 3. Restart DMA: Reset Count AND Address
        dma_channel_set_trans_count(dma_tx, TOTAL_SIZE, false);
        dma_channel_set_read_addr(dma_tx, tx_buffer, true);
        
        // Clear Data Ready (Handshake)
        gpio_put(PIN_DATA_READY, 0); 
    }
}

// Timer callback - runs at 1000Hz
bool timer_1khz_callback(struct repeating_timer *t) {
    // Read sensor (approximately 14 bytes * 8 bits / 4MHz = ~28us)
    ism330_data_t data;
    ism330_read_data(SPI_SENSOR_PORT, PIN_SENSOR_CS, &data);
    
    // Copy to volatile buffer
    sensor_data = data;
    new_data_ready = true;
    sample_count++;
    
    return true;  // Keep repeating
}



int main() {
    stdio_init_all();
    
    // GPIO Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_init(PIN_DATA_READY);
    gpio_set_dir(PIN_DATA_READY, GPIO_OUT);
    gpio_put(PIN_DATA_READY, 0);

    // === SPI1 Master (Sensor) Init ===
    spi_init(SPI_SENSOR_PORT, 4000000);  // 4MHz for sensor
    gpio_set_function(PIN_SENSOR_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENSOR_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENSOR_RX, GPIO_FUNC_SPI);
    // CS is manual GPIO
    gpio_init(PIN_SENSOR_CS);
    gpio_set_dir(PIN_SENSOR_CS, GPIO_OUT);
    gpio_put(PIN_SENSOR_CS, 1);

    // Initialize ISM330DHCX
    printf("[MAIN] Initializing ISM330DHCX...\n");
    bool sensor_ok = ism330_init(SPI_SENSOR_PORT, PIN_SENSOR_CS);
    if (!sensor_ok) {
        printf("[MAIN] ISM330DHCX init failed! Running in fallback mode.\n");
    } else {
        printf("[MAIN] ISM330DHCX ready!\n");
    }

    // === PIO SPI Slave Init ===
    uint offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_init(pio, sm, offset, PIN_MOSI, PIN_CS, PIN_SCK, PIN_MISO);
    
    printf("[MAIN] PIO SPI Slave Initialized (MISO=%d, MOSI=%d, SCK=%d, CS=%d)\n", PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS);

    // Init TX Buffer Header
    memcpy(tx_buffer, HEADER, 4);
    memset(tx_buffer + 4, 0, PAYLOAD_SIZE);

    // === DMA Init for PIO TX ===
    dma_tx = dma_claim_unused_channel(true);
    
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true)); // Wait for PIO TX FIFO
    channel_config_set_read_increment(&c, true);   // Increment read address (buffer)
    channel_config_set_write_increment(&c, false); // Fixed write address (PIO TXF)
    channel_config_set_ring(&c, false, 0);         // No ring buffer
    
    // Configure DMA channel
    dma_channel_configure(
        dma_tx,
        &c,
        &pio->txf[sm],  // Dest: PIO TX FIFO
        tx_buffer,      // Src: Buffer
        TOTAL_SIZE,     // Count
        false           // Don't start yet
    );
    
    // Enable DMA interrupt? No, we don't need interrupt if we just abort/restart on CS.
    // Actually, if buffer exhausts (200 bytes sent), DMA stops.
    // If master keeps clocking, PIO sends garbage or stalls?
    // PIO waits for TX FIFO. If empty, it stalls.
    // If Slave stalls, MISO holds last bit?
    // That's acceptable for overflow.
    
    // --- PIO IRQ Setup ---
    // Enable PIO IRQ 0 for this SM
    // 'irq 0' in PIO program maps to bit 0 of irq flags
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    // --- Pre-fill Header Manual ---
    // Seed with AA BB CC DD to debug connection
    tx_buffer[0] = 0xAA; tx_buffer[1] = 0xBB; tx_buffer[2] = 0xCC; tx_buffer[3] = 0xDD;
    
    // Start DMA immediately to fill FIFO
    dma_channel_start(dma_tx);
    
    printf("[MAIN] PIO SPI Slave Ready (IRQ Mode).\n");

    // === Start 1000Hz Timer ===
    struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_1khz_callback, NULL, &timer);

    uint32_t last_heartbeat = 0;
    uint32_t last_sample_count = 0;
    uint32_t last_rate_check = 0;

    while (true) {
        // Toggle LED based on CS state? No, heartbeat is better.
        // Update ISR count in buffer for visibility? Done in ISR.
        
        // Update buffer when new sensor data is available
        if (new_data_ready) {
            new_data_ready = false;
            
            // Pack data into buffer (DMA will send this)
            // Header is at 0-3. Floats start at 4.
            float *floats = (float*)(tx_buffer + 4); 
            
            floats[0] = (float)sample_count;
            floats[1] = sensor_data.temp;
            
            // Accel (floats 2-4)
            floats[2] = sensor_data.accel[0];
            floats[3] = sensor_data.accel[1];
            floats[4] = sensor_data.accel[2];
            
            floats[5] = sensor_data.gyro[0];
            floats[6] = sensor_data.gyro[1];
            floats[7] = sensor_data.gyro[2];
            
            // Signal RPi4 that data is ready (Rising Edge)
            gpio_put(PIN_DATA_READY, 1);
        }

        // === Continuously keep TX FIFO filled ===
        // DMA handles this automatically via DREQ from PIO!
        // No manual filling needed.

        // Drain PIO RX FIFO (we receive MOSI data)
        // We must read it to prevent PIO stalling on 'push'
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            pio_sm_get(pio, sm);
        }

        // Heartbeat LED (1Hz blink)
        uint32_t now = time_us_32();
        if (now - last_heartbeat > 500000) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_heartbeat = now;
        }

        // Rate check (every second, print actual rate)
        if (now - last_rate_check > 1000000) {
            uint32_t delta = sample_count - last_sample_count;
            // printf("[MAIN] Sample rate: %lu Hz\n", delta); // Disable print to avoid USB lag?
            last_sample_count = sample_count;
            last_rate_check = now;
        }
    }
}
