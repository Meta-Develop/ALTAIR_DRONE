/**
 * @file main.c
 * @brief Pico Actuators Firmware (Node B) - SPI Slave
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

// Actuator Drivers
#include "drivers/dshot.h"
#include "drivers/telemetry.h"

// --- Config ---
#define SPI_PORT spi0
// NOTE: Verify these pins match the actual SPI bus for Node B!
// If Node B shares the bus, MOSI/SCK/MISO are same, but CS is different.
// The constraints say CE1 (Pin 26) for Node B.
// But Pico pin mapping depends on wiring. 
// Assuming same wiring logic as Node A for the SPI peripheral pins on the Pico itself.
#define PIN_SPI_RX   16
#define PIN_SPI_CS   17 
#define PIN_SPI_SCK  18
#define PIN_SPI_TX   19

// Servo Config
#define SERVO_GPIO_BASE 0  // GPIO 0-5
#define NUM_SERVOS 6
#define SERVO_WRAP 39062   // 125MHz / 64 / 50Hz ~= 39062.5
#define SERVO_DIV  64.0f

// Payloads
#define PAYLOAD_TX_FLOATS 24
#define PAYLOAD_TX_BYTES (PAYLOAD_TX_FLOATS * sizeof(float))

#define PAYLOAD_RX_FLOATS 24
#define PAYLOAD_RX_BYTES (PAYLOAD_RX_FLOATS * sizeof(float))

// --- Globals ---
volatile uint32_t cs_event_count = 0;
float tx_data[PAYLOAD_TX_FLOATS]; 
uint8_t tx_buffer[PAYLOAD_TX_BYTES]; 
uint8_t rx_buffer[PAYLOAD_RX_BYTES];
float rx_data[PAYLOAD_RX_FLOATS];

int dma_tx;
int dma_rx;
dma_channel_config dma_tx_config;
dma_channel_config dma_rx_config;

// --- Servo Driver ---
void servo_init(void) {
    // Init GPIO 0-5
    for(int i=0; i<NUM_SERVOS; i++) {
        uint gpio = SERVO_GPIO_BASE + i;
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, SERVO_DIV);
        pwm_config_set_wrap(&config, SERVO_WRAP);
        
        pwm_init(slice_num, &config, true);
        // Start with 1.5ms center
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 2930); // ~1500us
    }
}

void servo_write(int idx, float val) {
    if (idx < 0 || idx >= NUM_SERVOS) return;
    
    // Clamp 0.0-1.0
    if (val < 0.0f) val = 0.0f;
    if (val > 1.0f) val = 1.0f;
    
    // Map 0.0-1.0 -> 1000us-2000us
    // Count = (us / 20000us) * WRAP
    // 1953 ticks = 1000us, 3906 ticks = 2000us
    
    uint16_t level = 1953 + (uint16_t)(val * 1953.0f);
    
    uint gpio = SERVO_GPIO_BASE + idx;
    pwm_set_gpio_level(gpio, level);
}

// --- Constants ---
#define MAGIC_HEADER 99.0f
#define ARM_LEN 0.25f
#define K_M 0.02f // Approx drag/thrust ratio
#define PI 3.14159265f

// --- ISR ---
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio == PIN_SPI_CS) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Transaction Start
            cs_event_count++;
            
            dma_channel_set_read_addr(dma_tx, tx_buffer, false);
            dma_channel_set_trans_count(dma_tx, PAYLOAD_TX_BYTES, false);
            
            dma_channel_set_write_addr(dma_rx, rx_buffer, false);
            dma_channel_set_trans_count(dma_rx, PAYLOAD_RX_BYTES, false);
            
            dma_channel_start(dma_rx);
            dma_channel_start(dma_tx);
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Transaction End
            dma_channel_abort(dma_tx);
            dma_channel_abort(dma_rx);
            
            memcpy(rx_data, rx_buffer, PAYLOAD_RX_BYTES);
            
            // --- DIRECT MODE ---
            // Servos are now handled by RPi4 via Dynamixel U2D2
            // We only handle Motors here.
            
            // Motors 0-5
            for(int i=0; i<6; i++) {
                float th = rx_data[i];
                if(th < 0.0f) th = 0.0f;
                if(th > 1.0f) th = 1.0f;
                dshot_write_throttle(i, th);
            }
            dshot_send_frame();
            
            // Clear SPI
            while (spi_is_readable(SPI_PORT)) (void)spi_get_hw(SPI_PORT)->dr;
        }
    }
}

int main() {
    stdio_init_all();

    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    
    // Actuator Init
    dshot_init();
    // Actuator Init
    dshot_init();
    // servo_init(); // Handled by RPi4
    telemetry_init();
    telemetry_init();

    // SPI Init (Slave)
    spi_init(SPI_PORT, 4000 * 1000); 
    spi_set_slave(SPI_PORT, true);
    gpio_set_function(PIN_SPI_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_CS,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX,  GPIO_FUNC_SPI);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // DMA TX Setup
    dma_tx = dma_claim_unused_channel(true);
    dma_tx_config = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&dma_tx_config, DMA_SIZE_8); 
    channel_config_set_dreq(&dma_tx_config, spi_get_dreq(SPI_PORT, true)); 
    channel_config_set_read_increment(&dma_tx_config, true);
    channel_config_set_write_increment(&dma_tx_config, false); 
    dma_channel_configure(dma_tx, &dma_tx_config, &spi_get_hw(SPI_PORT)->dr, tx_buffer, PAYLOAD_TX_BYTES, false);

    // DMA RX Setup
    dma_rx = dma_claim_unused_channel(true);
    dma_rx_config = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&dma_rx_config, DMA_SIZE_8); 
    channel_config_set_dreq(&dma_rx_config, spi_get_dreq(SPI_PORT, false)); // RX DREQ
    channel_config_set_read_increment(&dma_rx_config, false); // Read from SPI DR
    channel_config_set_write_increment(&dma_rx_config, true); // Write to buffer
    dma_channel_configure(dma_rx, &dma_rx_config, rx_buffer, &spi_get_hw(SPI_PORT)->dr, PAYLOAD_RX_BYTES, false);

    // CS Interrupt
    gpio_set_irq_enabled_with_callback(PIN_SPI_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &cs_irq_handler);

    bool led_state = false;

    // --- Loop ---
    while (true) {
        // Blink LED (Alive) - 2Hz Heartbeat
        bool led_on = (time_us_64() / 250000) % 2 == 0;
        gpio_put(25, led_on);
        
        // ... (rest of loop)
        tight_loop_contents();
        
        // Pack TX Data
        uint64_t t = time_us_64();
        tx_data[0] = (float)(t & 0xFFFFFFFF);       
        tx_data[1] = (float)(t >> 32); 
        
        // Zero out Sensors (2-13)
        for(int i=2; i<=13; i++) {
            tx_data[i] = 0.0f;
        }
        
        // RPMs (14-19)
        for(int i=0; i<6; i++) {
             tx_data[14+i] = telemetry_read_rpm(i);
        }
        
        // Update TX Buffer
        memcpy(tx_buffer, tx_data, PAYLOAD_TX_BYTES);

        // Debug Blink (Alive)
        gpio_put(25, led_state);
        led_state = !led_state;
        sleep_ms(2);
    }
}
