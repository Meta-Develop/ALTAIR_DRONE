#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// Sensor Driver
#include "drivers/ism330dhcx.h"

// =============================================================================
// DMA-Based SPI Slave Firmware with Real Sensor Reading
// Target: RP2350 (Pico 2) - Pico 2A (Sensors)
// 
// Architecture:
//   - SPI0 Slave: Responds to RPi4 master (GP16-19)
//   - SPI1 Master: Reads ISM330DHCX sensor (GP12-15)
//   - Timer: 1000Hz interrupt to read sensor
//   - DMA: Automatically fills SPI TX FIFO from buffer
// =============================================================================

// --- SPI0 Slave (Link to RPi4) ---
#define SPI_SLAVE_PORT spi0
#define PIN_SLAVE_RX   16  // MOSI from RPi
#define PIN_SLAVE_CS   17  // CS from RPi
#define PIN_SLAVE_SCK  18  // SCK from RPi
#define PIN_SLAVE_TX   19  // MISO to RPi

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

// DMA channels
static int dma_tx;

// TX buffer index - volatile for ISR access
static volatile uint8_t tx_idx = 0;

// Debug counter
static volatile uint8_t isr_cnt = 0;

// CS loopback IRQ handler - fires on RISING edge (Transaction Complete)
void cs_loopback_callback(uint gpio, uint32_t events) {
    if (gpio == PIN_CS_LOOPBACK && (events & GPIO_IRQ_EDGE_RISE)) {
        // CS went HIGH - transaction finished!
        // Prepare data for the NEXT transaction
        
        // Debug: Toggle LED
        gpio_xor_mask(1u << PIN_LED);
        
        // Update ISR count in buffer (Byte 4)
        isr_cnt++;
        tx_buffer[4] = isr_cnt;
        
        // Disable SPI to flush FIFOs (Safe now as CS is High)
        spi_get_hw(SPI_SLAVE_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
        
        // Pre-fill TX FIFO with next 8 bytes
        // No dummy byte needed if reset works properly when CS is High
        for (int i = 0; i < 8; i++) {
            while (!spi_is_writable(SPI_SLAVE_PORT)); 
            spi_get_hw(SPI_SLAVE_PORT)->dr = tx_buffer[i];
        }
        
        // Re-enable SPI
        spi_get_hw(SPI_SLAVE_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;
        
        // Set index to 8 - main loop keeps the rest filled
        tx_idx = 8;
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

// DMA TX completion handler - does nothing, CS loopback ISR handles restart
void dma_handler() {
    // Clear interrupt
    dma_hw->ints0 = 1u << dma_tx;
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

    // === SPI0 Slave (Link to RPi4) Init ===
    spi_init(SPI_SLAVE_PORT, 1000000);
    spi_set_slave(SPI_SLAVE_PORT, true);
    spi_set_format(SPI_SLAVE_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(PIN_SLAVE_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SLAVE_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SLAVE_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SLAVE_CS, GPIO_FUNC_SPI);

    // Init TX Buffer Header
    memcpy(tx_buffer, HEADER, 4);
    memset(tx_buffer + 4, 0, PAYLOAD_SIZE);

    // === DMA Init for SPI Slave TX ===
    dma_tx = dma_claim_unused_channel(true);
    
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(SPI_SLAVE_PORT, true));  // TX DREQ
    channel_config_set_read_increment(&c, true);   // Increment read address (buffer)
    channel_config_set_write_increment(&c, false); // Fixed write address (SPI DR)
    channel_config_set_ring(&c, false, 0);         // No ring buffer for read
    
    // Configure DMA channel
    dma_channel_configure(
        dma_tx,
        &c,
        &spi_get_hw(SPI_SLAVE_PORT)->dr,  // Write to SPI TX
        tx_buffer,                         // Read from buffer
        TOTAL_SIZE,                        // Transfer count
        false                              // Don't start yet
    );
    
    // Enable DMA interrupt to restart on completion
    dma_channel_set_irq0_enabled(dma_tx, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // === CS Loopback GPIO for Edge Detection ===
    // GP21 (Pin 27) is jumpered to GP17 (Pin 22) - CS signal
    gpio_init(PIN_CS_LOOPBACK);
    gpio_set_dir(PIN_CS_LOOPBACK, GPIO_IN);
    // Initial Pre-fill for first transaction
    // Debug: Store SR values in buffer to diagnose FIFO behavior
    
    // Read SR before writing: Check TFE (bit 0) and TNF (bit 1)
    // TFE=1 (Empty), TNF=1 (Not Full) -> SR should be 0x3 (or similar)
    uint32_t sr_before = spi_get_hw(SPI_SLAVE_PORT)->sr;
    tx_buffer[0] = (uint8_t)sr_before; 
    
    // Write 1 byte (0xAA)
    while (!spi_is_writable(SPI_SLAVE_PORT)); 
    spi_get_hw(SPI_SLAVE_PORT)->dr = 0xAA;
    
    // Read SR after 1 byte
    uint32_t sr_mid = spi_get_hw(SPI_SLAVE_PORT)->sr;
    tx_buffer[1] = (uint8_t)sr_mid;
    
    // Write remaining 7 bytes
    for (int i = 0; i < 7; i++) {
        while (!spi_is_writable(SPI_SLAVE_PORT)); 
        spi_get_hw(SPI_SLAVE_PORT)->dr = 0xBB + i;
    }
    
    // Read SR after 8 bytes
    // Should be Full? TNF=0? TFE=0?
    uint32_t sr_after = spi_get_hw(SPI_SLAVE_PORT)->sr;
    tx_buffer[2] = (uint8_t)sr_after;
    
    // Byte 3: Dummy/Padding
    tx_buffer[3] = 0xCC;
    
    // ISR Count (at offset 4)
    tx_buffer[4] = 0; // Will be incremented by ISR

    tx_idx = 8; // Main loop continues

    // Enable Rising Edge IRQ
    gpio_set_irq_enabled_with_callback(PIN_CS_LOOPBACK, GPIO_IRQ_EDGE_RISE, true, &cs_loopback_callback);
    printf("[MAIN] CS Rising Edge IRQ Enabled.\n");
    
    // Don't start DMA here - usage of DMA is mixed/suspended for this debug
    printf("[MAIN] Debug Mode: SPI status reporting.\n");

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
            // Note: Byte 0-3 are debug values, Byte 4 is ISR count.
            // Payload starts at Byte 5?
            // Original layout: Header (4 bytes) + Floats.
            // My debug layout: [SR1, SR2, SR3, CC, ISR_CNT, ...floats...]
            // So floats start at offset 5.
            
            float *floats = (float*)(tx_buffer + 8); // Align to 4 bytes? 
            // Wait, tx_buffer is uint8_t.
            // If I want floats aligned, I should use offset 8 (multiple of 4).
            // tx_buffer[0..4] used. 5,6,7 unused? 
            // Let's use offset 8.
            
            floats[0] = (float)sample_count;
            floats[1] = sensor_data.temp;
            
            // Accel (floats 2-4)
            floats[2] = sensor_data.accel[0];
            floats[3] = sensor_data.accel[1];
            floats[4] = sensor_data.accel[2];
            
            // Gyro (floats 5-7)
            floats[5] = sensor_data.gyro[0];
            floats[6] = sensor_data.gyro[1];
            floats[7] = sensor_data.gyro[2];
        }

        // === Continuously keep TX FIFO filled ===
        // ISR sets tx_idx = 8 (after filling 0-7)
        // Main loop fills FIFO from current index position
        while (spi_is_writable(SPI_SLAVE_PORT) && tx_idx < TOTAL_SIZE) {
            spi_get_hw(SPI_SLAVE_PORT)->dr = tx_buffer[tx_idx];
            tx_idx++;
        }

        // Drain RX FIFO (we don't need master's data)
        while (spi_is_readable(SPI_SLAVE_PORT)) {
            volatile uint8_t dummy = spi_get_hw(SPI_SLAVE_PORT)->dr;
            (void)dummy;
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
