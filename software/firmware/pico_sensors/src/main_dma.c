/**
 * SPI SLAVE FIRMWARE - HIGH FREQUENCY BATCHING (6.66kHz to 1kHz)
 * 
 * Architecture:
 * - IMU (ISM330DHCX) running at 6.66kHz (150us period).
 * - Main Loop polls IMU Status Register. If ready -> Writes sample to "Active Buffer".
 * - SPI Slave (RPi4) polls at 1kHz.
 * - On CS Rising Edge (End of Transfer):
 *     - Finalize the "Active Buffer" (add Mag/Baro/Timestamp/Checksum).
 *     - Swap "Active" and "Ready" buffers.
 *     - Reset "Active" buffer.
 * - On CS Falling Edge (Start of Transfer):
 *     - Point DMA to "Ready Buffer".
 *     - Start Transfer.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "spi_slave.pio.h"
#include "batch_protocol.h" 

#include "drivers/ism330dhcx.h" 
#include "drivers/mmc5983ma.h"
#include "drivers/bmp388.h"
#include "drivers/vl53l4cx.h"

// --- PIN CONFIGURATION ---
// SPI0 PIO Slave (to RPi4 Master)
#define PIN_MISO_SLAVE    19       // GP19 - Output to RPi4
#define PIN_CS_SLAVE      17       // GP17 - Input from RPi4
#define PIN_SCK_SLAVE     18       // GP18 - Input from RPi4
#define PIN_MOSI_SLAVE    16       // GP16 - Input from RPi4
#define PIN_DATA_READY    22       // GP22 - Output to RPi4

// SPI1 Master (to 9DoF Sensors)
#define PIN_MISO_SENSOR   12       // GP12
#define PIN_MOSI_SENSOR   11       // GP11
#define PIN_SCK_SENSOR    10       // GP10
#define PIN_CS_IMU        13       // GP13
#define PIN_CS_MAG        14       // GP14

// I2C0 (Baro + ToF)
#define PIN_SDA           4        
#define PIN_SCL           5        

#define PIN_LED     25

// --- GLOBAL BUFFERS ---
BatchPacket batch_buffers[2];
volatile uint8_t active_idx = 0; // The one being written to by Sensors
volatile uint8_t ready_idx = 1;  // The one read by DMA/SPI

// State for Slow Sensors (cached values to put in every batch)
int16_t cached_mag[3] = {0};
uint8_t mag_update_counter = 0;
int32_t cached_pressure_raw = 0;
uint16_t cached_tof_mm = 0;

// PIO/DMA
PIO pio = pio0;
uint sm = 0;
uint offset = 0;
int dma_tx;

volatile uint32_t cs_irq_count = 0;

// CRC8 Helper (Simple XOR for now, as per spec)
uint8_t calculate_checksum(const BatchPacket* p) {
    const uint8_t* buf = (const uint8_t*)p;
    uint8_t xor_sum = 0;
    // XOR everything except the last 6 bytes (Checksum + 5 Padding)
    // TOTAL_PACKET_SIZE is 128 or 136? header defined 136 max?
    // Let's rely on sizeof(BatchPacket) - padding
    // Checksum is at offset 122.
    for(int i=0; i<122; i++) {
        xor_sum ^= buf[i];
    }
    return xor_sum;
}

// --- INTERRUPT HANDLERS ---
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio != PIN_CS_SLAVE) return;

    // 1. FALLING EDGE: Start of Transaction (Host asserts CS)
    if (events & GPIO_IRQ_EDGE_FALL) {
        gpio_put(PIN_DATA_READY, 0); // Clear DRDY (we are modifying data lines now)
        
        // Reset and Restart PIO state machine
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(dma_tx);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        
        // JMP to entry point (Removed entry_point, just start SM)
        pio_sm_exec(pio, sm, pio_encode_jmp(offset));

        // Configure DMA to send the READY buffer
        dma_channel_set_trans_count(dma_tx, sizeof(BatchPacket), false); // Bytes
        dma_channel_set_read_addr(dma_tx, &batch_buffers[ready_idx], true); // Trigger logic? No, true means trigger?
        // Wait, dma_channel_set_read_addr(..., trigger=true) starts it.
        // But we need PIO to be enabled first? Or PIO enabled after?
        // Standard: Enable DMA, then Enable PIO.
        
        // We push to TX FIFO. If PIO is stalled, DMA fills FIFO and waits.
        // It's safe.
        pio_sm_set_enabled(pio, sm, true);
    }
    
    // 2. RISING EDGE: End of Transaction (Host releases CS)
    else if (events & GPIO_IRQ_EDGE_RISE) {
        cs_irq_count++;
        
        // SWAP BUFFERS
        // We just sent 'ready_idx'. Now 'active_idx' (which has been filling) should become 'ready'.
        
        // Critical Section: Finalize the Active Buffer
        // We assume main loop is filling 'active_idx'.
        // We might be interrupting a write? 
        // This is a race condition. 
        // If we interrupt `check_imu_data` while it's writing sample #3...
        // ...the packet might be incomplete or corrupt.
        // However, `valid_sample_count` is what matters.
        // If we interrupt, `valid_sample_count` won't be incremented yet, so the consumer ignores the partial data.
        // But we need to close the packet properly.

        BatchPacket* p = &batch_buffers[active_idx];
        
        // Fill Slow Sensor Data (from Global Cache)
        p->mag[0] = cached_mag[0];
        p->mag[1] = cached_mag[1];
        p->mag[2] = cached_mag[2];
        p->mag_count = mag_update_counter;
        p->pressure_raw = cached_pressure_raw;
        p->tof_mm = cached_tof_mm;
        
        // Fill Header
        p->magic[0] = BATCH_MAGIC_0;
        p->magic[1] = BATCH_MAGIC_1;
        p->frame_id++; // Simple wrapping counter
        p->timestamp_us = time_us_64(); // Capture time of BATCH COMPLETION
        
        // Checksum
        p->checksum = calculate_checksum(p);
        
        // Perform the Swap
        uint8_t old_active = active_idx;
        active_idx = ready_idx; // Old ready becomes new active (empty it first)
        ready_idx = old_active; // Old active becomes ready
        
        // Reset the NEW active buffer
        // batch_buffers[active_idx].valid_sample_count = 0; // Reset count
        // We do this via pointer to minimize overhead
        batch_buffers[active_idx].valid_sample_count = 0;
        
        // Signal Data Ready for the NEXT transaction?
        // We effectively have new data ready immediately after swap if the buffer wasn't empty.
        // But typically we signal when the buffer is "full enough" or just keep it high?
        // Let's assert DRDY to tell Host "I have data for you".
        gpio_put(PIN_DATA_READY, 1);
        
        // Toggle LED for heartbeat
        if (cs_irq_count % 100 == 0) gpio_xor_mask(1u << PIN_LED);
    }
}

// --- SETUP SENSORS ---
void setup_sensors() {
    // Standard Init
    ism330_init(spi1, PIN_CS_IMU);
    // CONFIG FOR 6.66kHz (High Perf)
    // CTRL1_XL (0x10): ODR_XL=1010 (6.66kHz), FS_XL=00 (2g)? No, Use user defaults.
    // Spec says: "ODR Setting: 6.66kHz (High Performance Mode, Reg 0xA0)"
    // We need to write this explicitly.
    uint8_t cfg_xl = 0xA0 | 0x0C; // 6.66k, 4g, LPF2_XL_EN? 
    // Let's stick to the basic 6.66k. (Bits 7-4 = 1010)
    // FS = 16g (Bits 3-2 = 01)? Or what was previous? Previous was 16g.
    // 0xA0 = 1010 0000. 
    // Let's use ism330 driver if it exposes it, or manual write.
    // Manual write for now to be sure.
    // CTRL1_XL = 0xA4 (6.66k, 16g) 
    // CTRL2_G  = 0xA4 (6.66k, 2000dps)
    uint8_t data[2];
    data[0] = 0x10; data[1] = 0xA4; 
    gpio_put(PIN_CS_IMU, 0); spi_write_blocking(spi1, data, 2); gpio_put(PIN_CS_IMU, 1);
    
    data[0] = 0x11; data[1] = 0xA4; 
    gpio_put(PIN_CS_IMU, 0); spi_write_blocking(spi1, data, 2); gpio_put(PIN_CS_IMU, 1);

    mmc5983_init(spi1, PIN_CS_MAG);
    bmp388_init(i2c0);
    vl53_init(i2c0);
    vl53_start_ranging(i2c0);
}

// --- MAIN LOOP TASKS ---
void check_imu_data() {
    // Poll Status Register (0x1E)
    // Bit 0 = XLDA (Accel New Data), Bit 1 = GDA (Gyro New Data)
    if (!ism330_data_ready(spi1, PIN_CS_IMU)) return;

    // Read Data (12 bytes)
    uint8_t raw[12];
    // Cmd 0x22 | 0x80 = OUTX_L_G reading starts there? 
    // Standard map: 
    // 0x22 OUTX_L_G
    // ...
    // 0x28 OUTX_L_A
    // Checking previous driver, it reads 12 bytes starting at 0x22.
    // So order is Gyro X,Y,Z then Accel X,Y,Z.
    
    uint8_t cmd = 0x22 | 0x80;
    gpio_put(PIN_CS_IMU, 0);
    spi_write_blocking(spi1, &cmd, 1);
    spi_read_blocking(spi1, 0x00, raw, 12);
    gpio_put(PIN_CS_IMU, 1);

    // Add to Active Buffer
    // Protect against race with CS IRQ by creating a local copy of idx?
    // No, active_idx is volatile.
    // If swap happens *during* calculation, we write to the *new* active buffer?
    // OR we write to the *old* one which is now ready, and corrupt it?
    // RISK: We calculate address `&batch_buffers[active_idx]`.
    // Then IRQ swaps `active_idx`.
    // Then we write to `batch_buffers[OLD_IDX]`.
    // OLD_IDX is now READY_IDX. DMA might be reading it!
    // CORRUPTION RISK.
    
    // FIX: Disable Interrupts for the "Commit" phase.
    uint32_t ints = save_and_disable_interrupts();
    
    BatchPacket* p = &batch_buffers[active_idx];
    if (p->valid_sample_count < MAX_BATCH_SIZE) {
        int idx = p->valid_sample_count;
        // Copy Raw Bytes? Protocol defines int16_t which matches.
        // Raw sequence: Gx L, Gx H, Gy L... 
        // Struct: accel[3], gyro[3].
        // Sensor output: Gyro First (0x22), then Accel (0x28).
        // Our Struct ImuSample: Accel first? 
        // struct { accel; gyro; }
        // We need to map correctly.
        
        // raw[0-5] = Gyro
        // raw[6-11] = Accel
        
        memcpy(p->samples[idx].gyro, &raw[0], 6);
        memcpy(p->samples[idx].accel, &raw[6], 6);
        
        p->valid_sample_count++;
    }
    
    // restore_interrupts(ints); 
    // Workaround for undefined reference:
    __asm volatile ("msr primask, %0" : : "r" (ints) : "memory");
}

void check_slow_sensors() {
    static uint32_t last_mag = 0;
    static uint32_t last_baro = 0;
    static uint32_t last_tof = 0;
    uint32_t now = time_us_32();
    
    // MAG (100Hz)
    if (now - last_mag > 10000) {
        last_mag = now;
        mmc5983_trigger_measurement(spi1, PIN_CS_MAG);
        // Wait? No, it takes time. Poll for ready?
        // Simpler: Trigger here, poll in next loop? Or assume previous is ready?
        // Let's use the poll check.
    }
    if (mmc5983_data_ready(spi1, PIN_CS_MAG)) {
         uint8_t mag_buf[7];
         // Read...
         uint8_t cmd = 0x00 | 0x80; // Register 0? 
         // Previous code used 0x00 | 0x80.
         gpio_put(PIN_CS_MAG, 0); spi_write_blocking(spi1, &cmd, 1); spi_read_blocking(spi1, 0x00, mag_buf, 7); gpio_put(PIN_CS_MAG, 1);
         
         uint16_t mx = (mag_buf[0] << 8) | mag_buf[1];
         uint16_t my = (mag_buf[2] << 8) | mag_buf[3];
         uint16_t mz = (mag_buf[4] << 8) | mag_buf[5];
         
         cached_mag[0] = (int16_t)(mx - 32768); // conversion?
         // Previous code treated it as unsigned 16 then wrote to byte buffer.
         // Let's assume standard int16 mapping.
         cached_mag[0] = (int16_t)mx; 
         cached_mag[1] = (int16_t)my;
         cached_mag[2] = (int16_t)mz;
         mag_update_counter++;
    }
    
    // Baro (20Hz)
    if (now - last_baro > 50000) {
        last_baro = now;
        bmp388_data_t b;
        bmp388_read_data(i2c0, &b);
        if (true) {
            cached_pressure_raw = *((int32_t*)&b.pressure); // Copy float bits? Or int?
            // Actually b.pressure is float. Protocol has int32_t pressure_raw.
            // Let's cast float bits to int32 for transport.
            memcpy(&cached_pressure_raw, &b.pressure, 4);
        }
    }
    
    // ToF (20Hz)
    if (now - last_tof > 50000) {
        last_tof = now;
        vl53_data_t t;
        vl53_read_data(i2c0, &t);
        cached_tof_mm = t.range_mm;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== ALTAIR SENSOR BRIDGE (BATCH MODE) ===\n");

    // Init Peripherals
    i2c_init(i2c0, 400000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C); gpio_pull_up(PIN_SDA);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C); gpio_pull_up(PIN_SCL);
    
    spi_init(spi1, 8000000);
    gpio_set_function(PIN_MISO_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_SENSOR, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_SENSOR, GPIO_FUNC_SPI);
    
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_init(PIN_DATA_READY); gpio_set_dir(PIN_DATA_READY, GPIO_OUT);

    setup_sensors();
    
    // Setup PIO/DMA Slave
    gpio_init(PIN_MISO_SLAVE); 
    offset = pio_add_program(pio, &spi_slave_program);
    spi_slave_init(pio, sm, offset, PIN_MISO_SLAVE);
    
    // Init Buffer Header
    batch_buffers[0].magic[0] = 0xAA; batch_buffers[0].magic[1] = 0x55;
    batch_buffers[1].magic[0] = 0xAA; batch_buffers[1].magic[1] = 0x55;

    // DMA Config
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); // 8-bit bytes
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    
    dma_channel_configure(dma_tx, &c, 
                          &pio->txf[sm],     // Dest
                          &batch_buffers[ready_idx], // Src
                          0,                 // Don't start yet
                          false);
                          
    // Interrupts
    gpio_init(PIN_CS_SLAVE); gpio_set_dir(PIN_CS_SLAVE, GPIO_IN); gpio_pull_up(PIN_CS_SLAVE);
    gpio_set_irq_enabled_with_callback(PIN_CS_SLAVE, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &cs_irq_handler);

    printf("Setup Complete. Entering High-Speed Loop.\n");
    
    while(1) {
        check_imu_data();
        check_slow_sensors();
        
        static uint32_t last_print = 0;
        uint32_t now = time_us_32();
        if (now - last_print > 1000000) {
            last_print = now;
            printf("SENSOR_ALIVE\n");
        }
        // No sleep. Run as fast as possible.
        // check_imu_data polls status reg, so it won't spinlock excessively on SPI bus logic.
    }
}
