#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"

// Sensor Driver
#include "drivers/ism330dhcx.h"

// =============================================================================
// High-Performance SPI Slave Firmware with Real Sensor Reading
// Target: RP2350 (Pico 2) - Pico 2A (Sensors)
// 
// Architecture:
//   - SPI0 Slave: Responds to RPi4 master (GP16-19)
//   - SPI1 Master: Reads ISM330DHCX sensor (GP12-15)
//   - Timer: 1000Hz interrupt to read sensor
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
#define PIN_LED 25

// --- Payload ---
// 96 bytes = 24 floats
// Layout: [Header 4B][Reserved 4B][Accel 12B][Gyro 12B][Mag 12B][Baro 8B][Range 4B][Padding]
#define PAYLOAD_SIZE 96

// Header bytes
static const uint8_t HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

// Global buffer (aligned for DMA)
static uint8_t tx_buffer[PAYLOAD_SIZE + 4] __attribute__((aligned(4)));

// Sensor data (volatile for ISR access)
static volatile ism330_data_t sensor_data;
static volatile bool new_data_ready = false;
static volatile uint32_t sample_count = 0;

// DMA
static int dma_tx;
static dma_channel_config dma_tx_config;

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

// Configure fast input for SPI slave pins
void configure_fast_spi_slave() {
    // Enable input sync bypass for lower latency on slave pins
    hw_write_masked(&padsbank0_hw->io[PIN_SLAVE_SCK], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS);
    hw_write_masked(&padsbank0_hw->io[PIN_SLAVE_RX], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS);
    hw_write_masked(&padsbank0_hw->io[PIN_SLAVE_CS], 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS, 
        PADS_BANK0_GPIO0_INPUT_SYNC_BYPASS_BITS);
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
        // Continue anyway - will output zeros
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

    configure_fast_spi_slave();

    // === DMA Init ===
    dma_tx = dma_claim_unused_channel(true);
    dma_tx_config = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&dma_tx_config, DMA_SIZE_8);
    channel_config_set_dreq(&dma_tx_config, spi_get_dreq(SPI_SLAVE_PORT, true));
    channel_config_set_read_increment(&dma_tx_config, true);
    channel_config_set_write_increment(&dma_tx_config, false);

    // Init TX Buffer Header
    memcpy(tx_buffer, HEADER, 4);
    memset(tx_buffer + 4, 0, PAYLOAD_SIZE);

    // === Start 1000Hz Timer ===
    struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_1khz_callback, NULL, &timer);
    printf("[MAIN] 1000Hz timer started.\n");

    uint32_t last_heartbeat = 0;
    uint32_t last_sample_count = 0;
    uint32_t last_rate_check = 0;

    while (true) {
        // Check for new sensor data
        if (new_data_ready) {
            new_data_ready = false;
            
            // Pack data into buffer
            // Float layout: [0]=header1, [1]=reserved, [2-4]=accel, [5-7]=gyro, [8-10]=mag...
            float *floats = (float*)(tx_buffer + 4);
            
            // Reserved (timestamp or counter)
            floats[0] = (float)sample_count;
            floats[1] = sensor_data.temp;
            
            // Accel (floats 2-4, bytes 12-23)
            floats[2] = sensor_data.accel[0];
            floats[3] = sensor_data.accel[1];
            floats[4] = sensor_data.accel[2];
            
            // Gyro (floats 5-7, bytes 24-35)
            floats[5] = sensor_data.gyro[0];
            floats[6] = sensor_data.gyro[1];
            floats[7] = sensor_data.gyro[2];
            
            // Mag, Baro, Range - zeros for now
            // (Would add MMC5983MA and BMP388 drivers later)
            
            // === Setup DMA for Slave Response ===
            dma_channel_configure(
                dma_tx,
                &dma_tx_config,
                &spi_get_hw(SPI_SLAVE_PORT)->dr,
                tx_buffer,
                PAYLOAD_SIZE + 4,
                false
            );

            // Arm DMA
            dma_channel_start(dma_tx);

            // Signal Data Ready to RPi4
            sleep_us(10);  // Small delay for DMA to prime FIFO
            gpio_put(PIN_DATA_READY, 1);

            // Wait for transfer (blocking)
            dma_channel_wait_for_finish_blocking(dma_tx);

            // Clear handshake
            gpio_put(PIN_DATA_READY, 0);
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
            printf("[MAIN] Sample rate: %lu Hz\n", delta);
            last_sample_count = sample_count;
            last_rate_check = now;
        }
    }
}
