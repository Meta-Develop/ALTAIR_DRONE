// PIO + DMA SPI Slave firmware for Pico 2 (RP2350)
// Uses PIO for SPI slave mode 3, DMA for efficient data transfer

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#include "board_config.h"
#include "spi_slave.pio.h"
#include "sensor_packet.h"
#include "ism330.h"
#include "mmc5983.h"
#include "bmp388.h"
#include "vl53l4cx.h"

// Aliases for legacy compatibility (PIO function uses old names)
#define PIN_MOSI  PIN_SPI_SLAVE_MOSI
#define PIN_CS    PIN_SPI_SLAVE_CS
#define PIN_SCK   PIN_SPI_SLAVE_SCK
#define PIN_MISO  PIN_SPI_SLAVE_MISO
#define PIN_SENS_CS PIN_IMU_CS
#define I2C_PORT  I2C_SENS_PORT
#define PIN_SDA   PIN_I2C_SDA
#define PIN_SCL   PIN_I2C_SCL
#define LED_PIN   PIN_LED

// PIO and DMA
static PIO pio = pio0;
static uint sm = 0;
static int dma_tx_chan = -1;
static int dma_rx_chan = -1;

// Double buffering for SPI
static SensorPacket tx_buffer[2];
static uint8_t rx_buffer[sizeof(SensorPacket)];  // Dummy RX
static volatile int active_buffer = 0;
static volatile uint32_t packet_seq = 0;

// Shared sensor data (updated by Core 0)
static volatile SensorPacket shared_packet;
static auto_init_mutex_t batch_mutex;

// Sensor Objects
ISM330DHCX* imu = nullptr;
MMC5983MA* mag = nullptr;
BMP388* baro = nullptr;
VL53L4CX* tof = nullptr;

uint32_t calculate_checksum(volatile SensorPacket* pkt) {
    uint32_t checksum = 0;
    const uint8_t* ptr = (const uint8_t*)pkt;
    for (size_t i = 0; i < sizeof(SensorPacket) - sizeof(uint32_t); ++i) {
        checksum ^= ptr[i];
    }
    return checksum;
}

void init_pio_spi_slave() {
    uint offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_program_init(pio, sm, offset, PIN_MOSI, PIN_CS, PIN_SCK, PIN_MISO);
}

void init_dma() {
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_channel_config tx_config = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));
    
    dma_channel_configure(dma_tx_chan, &tx_config, &pio->txf[sm], &tx_buffer[0], sizeof(SensorPacket), false);
    
    dma_rx_chan = dma_claim_unused_channel(true);
    dma_channel_config rx_config = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, pio_get_dreq(pio, sm, false));
    
    dma_channel_configure(dma_rx_chan, &rx_config, rx_buffer, &pio->rxf[sm], sizeof(SensorPacket), false);
}

void start_dma_transfer() {
    const uint8_t* tx_ptr = (const uint8_t*)&tx_buffer[active_buffer];
    dma_channel_set_read_addr(dma_tx_chan, tx_ptr, false);
    dma_channel_set_write_addr(dma_rx_chan, rx_buffer, false);
    dma_channel_set_trans_count(dma_tx_chan, sizeof(SensorPacket), false);
    dma_channel_set_trans_count(dma_rx_chan, sizeof(SensorPacket), false);
    dma_start_channel_mask((1u << dma_tx_chan) | (1u << dma_rx_chan));
}

void core1_entry() {
    init_pio_spi_slave();
    init_dma();
    
    memset(&tx_buffer[0], 0, sizeof(SensorPacket));
    memset(&tx_buffer[1], 0, sizeof(SensorPacket));
    
    tx_buffer[active_buffer].packet_seq = 0;
    tx_buffer[active_buffer].valid_imu_count = 0;
    tx_buffer[active_buffer].checksum = calculate_checksum(&tx_buffer[active_buffer]);
    
    start_dma_transfer();
    
    while (true) {
        // Wait for DMA to finish previous transfer
        while (dma_channel_is_busy(dma_tx_chan) || dma_channel_is_busy(dma_rx_chan)) {
            tight_loop_contents();
        }
        
        int next_buffer = (active_buffer + 1) % 2;
        
        // Critical Section: Move shared batch to TX buffer and Reset
        mutex_enter_blocking(&batch_mutex);
        memcpy(&tx_buffer[next_buffer], (void*)&shared_packet, sizeof(SensorPacket));
        shared_packet.valid_imu_count = 0; // Reset batch
        mutex_exit(&batch_mutex);
        
        tx_buffer[next_buffer].packet_seq = packet_seq++;
        tx_buffer[next_buffer].checksum = calculate_checksum(&tx_buffer[next_buffer]);
        active_buffer = next_buffer;
        
        start_dma_transfer();
    }
}

void scan_i2c_bus() {
    printf("\nI2C Bus Scan (I2C0, GP4/GP5)...\n");
    bool found_any = false;
    for (int addr = 0x08; addr < 0x78; ++addr) {
        int ret;
        uint8_t rxdata;
        ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        if (ret >= 0) {
            printf("Found I2C device at 0x%02X\n", addr);
            found_any = true;
        }
    }
    if (!found_any) {
        printf("No I2C devices found!\n");
    }
    printf("Scan complete.\n");
}

void init_sensors() {
    // SPI Init (Sensor Bus)
    spi_init(SPI_SENS_PORT, SPI_SENS_BAUDRATE);
    gpio_set_function(PIN_SENS_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENS_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENS_MISO, GPIO_FUNC_SPI);
    
    // I2C Init (Sensor Bus)
    i2c_init(I2C_PORT, I2C_SENS_BAUDRATE);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    
    sleep_ms(100); // Wait for sensor power up
    scan_i2c_bus();

    // IMU
    imu = new ISM330DHCX(SPI_SENS_PORT, PIN_SENS_CS);
    if (!imu->init()) printf("IMU Init Failed!\n");
    else printf("IMU Init Success!\n");
    
    // Mag
    mag = new MMC5983MA(SPI_SENS_PORT, PIN_MAG_CS);
    if (!mag->init()) printf("Mag Init Failed!\n");
    else printf("Mag Init Success!\n");
    
    // Baro
    baro = new BMP388(I2C_PORT, 0x77); 
    if (!baro->init()) {
        baro = new BMP388(I2C_PORT, 0x76);
        if(!baro->init()) printf("Baro Init Failed (0x76)!\n");
        else printf("Baro Init Success (0x76)!\n");
    } else {
        printf("Baro Init Success (0x77)!\n");
    }
    
    // ToF
    tof = new VL53L4CX(I2C_PORT);
    if (!tof->init()) {
        printf("ToF Init Failed!\n");
    } else {
        printf("ToF Init Success!\n");
    }
}

int main() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
    
    stdio_init_all();
    init_sensors();
    multicore_launch_core1(core1_entry);
    
    uint64_t last_mag_time = 0;
    uint64_t last_baro_time = 0;
    uint64_t last_tof_time = 0;

    while (true) {
        uint64_t now = time_us_64();
        
        // IMU Batching
        if (imu) {
            // Check Data Ready (Status Reg Bit 0=XL, Bit 1=G)
            uint8_t status = imu->read_reg(ISM330_REG_STATUS);
            if (status & 0x03) { // XL or G ready
                float accel[3], gyro[3];
                imu->read_data(accel, gyro);
                
                mutex_enter_blocking(&batch_mutex);
                if (shared_packet.valid_imu_count < IMU_BATCH_SIZE) {
                    int idx = shared_packet.valid_imu_count;
                    shared_packet.imu_samples[idx].timestamp_us = now;
                    shared_packet.imu_samples[idx].accel[0] = accel[0];
                    shared_packet.imu_samples[idx].accel[1] = accel[1];
                    shared_packet.imu_samples[idx].accel[2] = accel[2];
                    shared_packet.imu_samples[idx].gyro[0] = gyro[0];
                    shared_packet.imu_samples[idx].gyro[1] = gyro[1];
                    shared_packet.imu_samples[idx].gyro[2] = gyro[2];
                    shared_packet.valid_imu_count++;
                }
                mutex_exit(&batch_mutex);
            }
        }

        // Mag
        if (mag && (now - last_mag_time >= MAG_POLL_INTERVAL_US)) {
            float m[3];
            mag->read_data(m);
            shared_packet.mag[0] = m[0];
            shared_packet.mag[1] = m[1];
            shared_packet.mag[2] = m[2];
            shared_packet.mag_seq++;
            last_mag_time = now;
        }
        
        // Baro
        if (baro && (now - last_baro_time >= BARO_POLL_INTERVAL_US)) {
            float press, temp;
            if(baro->read_data(&press, &temp)) {
                shared_packet.pressure = press;
                shared_packet.temperature = temp;
                shared_packet.baro_seq++;
            }
            last_baro_time = now;
        }
        
        // ToF
        if (tof && (now - last_tof_time >= TOF_POLL_INTERVAL_US)) {
            uint16_t dist;
            if(tof->read_distance(&dist)) {
                // shared_packet.distance_mm = dist;
                shared_packet.tof_seq++;
            }
            last_tof_time = now;
        }

        gpio_xor_mask(1u << LED_PIN);
        sleep_us(MAIN_LOOP_DELAY_US);
    }
}
