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
#include "spi_slave.pio.h"

// --- Config ---
// Motor Pins: GP0-GP5 (defined in dshot.c)

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
ActuatorPacket rx_pkt;
TelemetryPacket tx_pkt;

uint tlm_req_idx = 0;

int main() {
    stdio_init_all();
    
    // Init DShot (GP0-GP5)
    // Uses dshot_bidir.pio (Currently TX implemented, RX hooks ready)
    dshot_init();
    
    // Init SPI Slave (Placeholder for full SPI logic)
    // ...
    
    // Init TX Packet Header
    tx_pkt.magic[0] = 0xCA; tx_pkt.magic[1] = 0xFE;
    
    while(true) {
        // Loop Rate: ~1kHz ideally synced to SPI
        sleep_us(1000);
        
        // 1. Process Motors
        for(int i=0; i<6; i++) {
            uint16_t val = rx_pkt.throttle[i];
            
            // Safety: Disarm check
            if (val == 0 && (rx_pkt.flags & 1)) val = 48; // Min spin if armed
             
            // Request Telemetry from ONE motor per frame (Round Robin)
            // This prevents overloading the loop if we were decoding 6 frames.
            // AM32 supports polling.
            bool telem = (i == tlm_req_idx);
            
            dshot_write_throttle(i, val, telem);
        }
        
        // Trigger Output
        // dshot_send_frame(); // Removed: Write triggers immediate send

        
        // 2. Read Telemetry (Placeholder)
        // In future: Read PIO RX FIFO here.
        // For now: We rely on TX to verify spinning.
        
        // Update Round Robin Index
        tlm_req_idx = (tlm_req_idx + 1) % 6;
        
        // TODO: Update SPI MISO buffer with tx_pkt
    }
}
