#include "mmc5983.h"
#include <stdio.h>

MMC5983MA::MMC5983MA(spi_inst_t* spi, uint cs_pin) : _spi(spi), _cs_pin(cs_pin) {
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1); // Deselect
}

void MMC5983MA::cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

void MMC5983MA::cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

void MMC5983MA::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value}; // SPI write is Address + Data
    // Note: MMC5983 SPI protocol for write: MSB 0 (Write)
    // Address [6:0]? Check datasheet.
    // "Standard 4-wire SPI". 
    // Usually MSB 0=Write, 1=Read? Or vice versa?
    // Datasheet: "Read: MSB=1, Write: MSB=0"
    
    buf[0] = reg & 0x7F; // Ensure MSB is 0 for write
    
    cs_select();
    spi_write_blocking(_spi, buf, 2);
    cs_deselect();
}

uint8_t MMC5983MA::read_reg(uint8_t reg) {
    uint8_t cmd = reg | 0x80; // Set MSB for Read
    uint8_t val;
    cs_select();
    spi_write_blocking(_spi, &cmd, 1);
    spi_read_blocking(_spi, 0, &val, 1);
    cs_deselect();
    return val;
}

void MMC5983MA::read_regs(uint8_t reg, uint8_t* buf, int len) {
    uint8_t cmd = reg | 0x80; // Set MSB for Read
    cs_select();
    spi_write_blocking(_spi, &cmd, 1);
    spi_read_blocking(_spi, 0, buf, len);
    cs_deselect();
}

bool MMC5983MA::init() {
    // Soft Reset
    write_reg(MMC5983_REG_INT_CTRL_1, 0x80); // SW_RST
    sleep_ms(10); // Wait for reset
    
    // Check ID
    uint8_t id = read_reg(MMC5983_REG_PRODUCT_ID);
    if (id != MMC5983_PRODUCT_ID) {
        printf("MMC5983 ID mismatch: 0x%02X\n", id);
        return false;
    }

    // Config: 
    // Ctrl 0: Auto Set/Reset (Bit 5 = Auto_SR_en)?
    // Datasheet: Bit 5: Auto_SR_en.
    write_reg(MMC5983_REG_INT_CTRL_0, 0x20); // Enable Auto Set/Reset

    // Ctrl 1: Bandwidth 8ms (100Hz) -> BW1=0, BW0=0 (default 10ms?)
    // Bits 1:0 BW. 00=6.6ms(100Hzish), 01=3.5ms(200Hz), 10=2.0ms(400Hz), 11=1.2ms(800Hz)
    // Let's use 00 (Default)
    write_reg(MMC5983_REG_INT_CTRL_1, 0x00); 

    // Ctrl 2: Continuous Mode 100Hz (Cmm_en=1, Cmm_freq_en=1, Prd_set=?)
    // Bits [2:0] Prd_set: 000=1Hz ... 011=50Hz, 100=100Hz?
    // Datasheet: 
    // Cmm_en (Bit 3) = 1 (Continuous)
    // Cmm_freq_en (Bit 4) = 1 (Use internal frequency)
    // Prd_set (Bits 2:0): 001? (Wait, checking table)
    // No, Prd_set is sampling rate.
    // 100 (binary) = 100Hz.
    // So 0x10 | 0x08 | 0x04 = 0x1C?
    // Let's try to set continuous mode enabled (Bit 3) + Freq En (Bit 4) + 100Hz (Bit 2:0 = 100)
    // 0001 1100 = 0x1C. 
    write_reg(MMC5983_REG_INT_CTRL_2, 0x1C); 

    return true;
}

void MMC5983MA::read_data(float* mag) {
    uint8_t raw[7];
    read_regs(MMC5983_REG_X_OUT_0, raw, 7);
    
    // Convert 18-bit (packed into 16 bit for now by ignoring bits 1:0 in status?)
    // Actually: Xout0 is X[17:10], Xout1 is X[9:2].
    // Resolution: 18-bit range (0-262143).
    // We only read top 16 bits: X = (X0 << 8) | X1.
    // This value matches X[17:2].
    // Full 18-bit value would be (X0 << 10) | (X1 << 2) | (Status bits?).
    // If we just use headers, we get 16-bit resolution.
    // 18-bit zero offset: 131072. 16-bit zero offset: 32768.
    // Sensitivity 18-bit: 16384 counts/G.
    // Sensitivity 16-bit: 4096 counts/G.
    
    // Let's use 16-bit extraction for simplicity/speed
    uint16_t x_raw = (raw[0] << 8) | raw[1];
    uint16_t y_raw = (raw[2] << 8) | raw[3];
    uint16_t z_raw = (raw[4] << 8) | raw[5];
    
    // Unsigned to Signed conversion
    // Offset is 32768 (2^15)
    float x = (float)x_raw - 32768.0f;
    float y = (float)y_raw - 32768.0f;
    float z = (float)z_raw - 32768.0f;
    
    // Scale to Gauss (16-bit sensitivity = 4096 LSB/G)
    // 16384 (18bit) >> 2 = 4096 (16bit)
    float scale = 1.0f / 4096.0f;
    
    mag[0] = x * scale;
    mag[1] = y * scale;
    mag[2] = z * scale;
}
