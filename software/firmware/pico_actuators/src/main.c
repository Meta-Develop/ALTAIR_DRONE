/**
 * ALTAIR Drone - Pico 2B (Actuators) Firmware
 * Controls 6 ESCs via Bidirectional DShot.
 * Wiring: Signal + GND only.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "drivers/dshot.h"
#include "spi_slave_duplex.pio.h"

// --- Config ---
#define PIN_MISO 19
#define PIN_CS   17
// PIN_SCK   18 (Hardcoded in PIO)
#define PIN_MOSI 16

// --- Protocol Structs ---
#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2]; // 0xBA, 0xBE
    uint16_t throttle[6]; // 0-2047
    uint8_t flags; // 1=Arm, 2=Beep
    uint8_t checksum; 
} ActuatorPacket;

typedef struct {
    uint8_t magic[2]; // 0xCA, 0xFE
    uint16_t rpm[6];
    uint16_t voltage[6];
    uint16_t current[6];
    uint8_t temp[6];
    uint8_t status;
    uint8_t checksum;
} TelemetryPacket;
#pragma pack(pop)

// Globals
volatile ActuatorPacket rx_pkt; // volatile as updated by DMA/IRQ
TelemetryPacket tx_pkt;

uint8_t rx_buffer[sizeof(ActuatorPacket) + 32]; // Buffer for raw reception (+padding)

// PIO/DMA State
PIO pio_spi = pio0; // Using PIO0 (DShot uses PIO0/1... conflict? DShot uses pio0 and pio1)
// Check DShot Resource Usage:
// DShot init claims SMs.
// We need 1 SM for SPI.
// DShot uses 6 SMs total (4 on PIO0, 2 on PIO1).
// PIO0 has 4 SMs. All used by Motors 0-3!
// **CRITICAL RESOURCE CONFLICT**
// Motors 0-3 -> PIO0 SM 0,1,2,3.
// Motors 4-5 -> PIO1 SM 0,1.
// We must put SPI Slave on PIO1 (SM 2 or 3).

PIO pio_dshot_lower = pio0;
PIO pio_dshot_upper = pio1;
PIO pio_slave = pio1; // Use PIO1 for SPI

uint sm_slave = 2; // SM 2 on PIO1
uint offset_slave;

int dma_tx;
int dma_rx;

uint tlm_req_idx = 0;

// --- IRQ Handler ---
void cs_irq_handler(uint gpio, uint32_t events) {
    if (gpio != PIN_CS) return;

    if (events & GPIO_IRQ_EDGE_FALL) {
        // --- Transaction START ---
        // 1. Reset PIO/DMA
        pio_sm_set_enabled(pio_slave, sm_slave, false);
        pio_sm_clear_fifos(pio_slave, sm_slave);
        pio_sm_restart(pio_slave, sm_slave);
        pio_sm_exec(pio_slave, sm_slave, pio_encode_jmp(offset_slave));
        
        // 2. Restart DMA Channels
        dma_channel_abort(dma_tx);
        dma_channel_abort(dma_rx);
        
        // RX: Read from PIO RX FIFO to rx_buffer
        dma_channel_transfer_to_buffer_now(dma_rx, rx_buffer, sizeof(rx_buffer));
        
        // TX: Write tx_pkt to PIO TX FIFO
        // Note: tx_pkt should be ready.
        // Update Checksum just in case
        // (Checksum calculation omitted for speed, assume logic does it)
        dma_channel_transfer_from_buffer_now(dma_tx, &tx_pkt, sizeof(TelemetryPacket));
        
        // 3. Enable PIO
        pio_sm_set_enabled(pio_slave, sm_slave, true);
        
        gpio_put(25, 1); // LED ON at Start
    }
    else if (events & GPIO_IRQ_EDGE_RISE) {
        // --- Transaction END ---
        gpio_put(25, 0); // LED OFF at End

        // Check if we received enough data?
        // Copy rx_buffer to rx_pkt if valid magic
        if (rx_buffer[0] == 0xBA && rx_buffer[1] == 0xBE) {
            memcpy((void*)&rx_pkt, rx_buffer, sizeof(ActuatorPacket));
        }
        
        // Prepare TX packet for NEXT time?
        // tx_pkt matches Global state (Motors).
        // Update header/checksum
        tx_pkt.magic[0] = 0xCA; tx_pkt.magic[1] = 0xFE;
        // Checksum calculation
        uint8_t sum = 0;
        uint8_t* b = (uint8_t*)&tx_pkt;
        for(int i=0; i<sizeof(TelemetryPacket)-1; i++) sum ^= b[i];
        tx_pkt.checksum = sum;
    }
}

int main() {
    stdio_init_all();
    
    // Init DShot (GP0-GP5)
    // dshot_init() claims PIO0(0-3) and PIO1(0-1).
    dshot_init(); 
    
    // Init SPI Slave (PIO1 SM2)
    offset_slave = pio_add_program(pio_slave, &spi_slave_duplex_program);
    spi_slave_duplex_init(pio_slave, sm_slave, offset_slave, PIN_MISO, PIN_MOSI);
    
    // DMA Setup
    dma_tx = dma_claim_unused_channel(true);
    dma_rx = dma_claim_unused_channel(true);
    
    dma_channel_config c_tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_tx, true);
    channel_config_set_write_increment(&c_tx, false);
    channel_config_set_dreq(&c_tx, pio_get_dreq(pio_slave, sm_slave, true)); // TX DREQ
    dma_channel_configure(dma_tx, &c_tx, &pio_slave->txf[sm_slave], NULL, 0, false);
    
    dma_channel_config c_rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_rx, true);
    channel_config_set_write_increment(&c_rx, false);
    channel_config_set_dreq(&c_rx, pio_get_dreq(pio_slave, sm_slave, false)); // RX DREQ
    dma_channel_configure(dma_rx, &c_rx, rx_buffer, &pio_slave->rxf[sm_slave], 0, false);
    
    // IRQ Setup
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_IN); gpio_pull_up(PIN_CS);
    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &cs_irq_handler);

    // Debug LED
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);


    // Initial Packet
    tx_pkt.magic[0] = 0xCA; tx_pkt.magic[1] = 0xFE;

    while(true) {
        // Loop Rate: ~1kHz ideally synced to SPI
        sleep_us(1000);
        printf("ACTUATOR_ALIVE\n");
        
        // 1. Process Motors
        for(int i=0; i<6; i++) {
            uint16_t val = rx_pkt.throttle[i];
            
            // Safety: Disarm check
            if (val == 0 && (rx_pkt.flags & 1)) val = 48; // Min spin if armed
             
            // Request Telemetry from ONE motor per frame (Round Robin)
            bool telem = (i == tlm_req_idx);
            
            dshot_write_throttle(i, val, telem);
        }
        
        // Update Round Robin Index
        tlm_req_idx = (tlm_req_idx + 1) % 6;
    }
}
