#include "mmc5983ma.h"
#include <stdio.h>

// --- SPI Helpers ---
static uint8_t mmc5983_read_reg(spi_inst_t *spi, uint cs_pin, uint8_t reg) {
    // SPI Read: Bit 7 = 1
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2];
    
    gpio_put(cs_pin, 0);
    spi_write_read_blocking(spi, tx, rx, 2);
    gpio_put(cs_pin, 1);
    
    return rx[1];
}

static void mmc5983_write_reg(spi_inst_t *spi, uint cs_pin, uint8_t reg, uint8_t val) {
    // SPI Write: Bit 7 = 0
    uint8_t tx[2] = {reg & 0x7F, val};
    
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, tx, 2);
    gpio_put(cs_pin, 1);
}

static void mmc5983_read_burst(spi_inst_t *spi, uint cs_pin, uint8_t start_reg, uint8_t *buf, size_t len) {
    uint8_t cmd = start_reg | 0x80;  // Read + auto-increment
    
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buf, len);
    gpio_put(cs_pin, 1);
}

// --- Public API ---
bool mmc5983_init(spi_inst_t *spi, uint cs_pin) {
    // CS Pin Setup
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);  // Deselect
    sleep_ms(10);
    
    // Read Product ID
    uint8_t prod_id = mmc5983_read_reg(spi, cs_pin, MMC5983_PRODUCT_ID);
    printf("[MMC5983] Product ID: 0x%02X (Expected 0x%02X)\n", prod_id, MMC5983_PRODUCT_ID_VAL);
    
    if (prod_id != MMC5983_PRODUCT_ID_VAL) {
        printf("[MMC5983] Init FAILED - Wrong Product ID!\n");
        return false;
    }
    
    // Software Reset
    mmc5983_write_reg(spi, cs_pin, MMC5983_CTRL1, 0x80);  // SW_RST bit
    sleep_ms(10);
    
    // Enable continuous measurement mode at Max Speed
    // CTRL1: Set BW to 800Hz (Bist 1:0 = 11 -> 0x03)
    mmc5983_write_reg(spi, cs_pin, MMC5983_CTRL1, MMC5983_BW_800HZ);
    
    // CTRL2: Cmm_en (Bit 3) + Cm_freq (Bits 2:0)
    // Freq 1000Hz = 111 (0x07). Total 0x08 | 0x07 = 0x0F
    mmc5983_write_reg(spi, cs_pin, MMC5983_CTRL2, 0x0F); // 0x08 (CMM) | 0x07 (1000Hz) ?? 
    // Note: Datasheet says 0x07 is 1000Hz.
    
    // CTRL0: Set Auto_SR_en (Bit 5)? No, keep simple. 
    // Just enable CMM.
    
    printf("[MMC5983] Init SUCCESS (Continuous Mode 1000Hz)\n");
    return true;
}

// Deprecated for Continuous Mode
void mmc5983_trigger_measurement(spi_inst_t *spi, uint cs_pin) {
    mmc5983_write_reg(spi, cs_pin, MMC5983_CTRL0, MMC5983_CMD_TM_M);
}

bool mmc5983_data_ready(spi_inst_t *spi, uint cs_pin) {
    uint8_t status = mmc5983_read_reg(spi, cs_pin, MMC5983_STATUS);
    // Bit 0 = Meas_M_Done
    return (status & 0x01) != 0;
}

void mmc5983_read_data(spi_inst_t *spi, uint cs_pin, mmc5983_data_t *data) {
    uint8_t buf[7];  // XOUT0, XOUT1, YOUT0, YOUT1, ZOUT0, ZOUT1, XYZOUT2
    
    mmc5983_read_burst(spi, cs_pin, MMC5983_XOUT0, buf, 7);
    
    // Combine into 18-bit unsigned values
    // X: [17:10] from XOUT0, [9:2] from XOUT1, [1:0] from XYZOUT2[7:6]
    uint32_t x_raw = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03);
    uint32_t y_raw = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x03);
    uint32_t z_raw = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x03);
    
    // Convert to signed (centered at 131072)
    int32_t x_signed = (int32_t)x_raw - MMC5983_OFFSET;
    int32_t y_signed = (int32_t)y_raw - MMC5983_OFFSET;
    int32_t z_signed = (int32_t)z_raw - MMC5983_OFFSET;
    
    // Convert to uT
    data->mag[0] = (float)x_signed * MMC5983_SENS_UT;
    data->mag[1] = (float)y_signed * MMC5983_SENS_UT;
    data->mag[2] = (float)z_signed * MMC5983_SENS_UT;
}
