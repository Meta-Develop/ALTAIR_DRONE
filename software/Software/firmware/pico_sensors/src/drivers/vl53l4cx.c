#include "vl53l4cx.h"
#include <stdio.h>

// I2C helpers for 16-bit register addressing
static bool vl53_write_reg8(i2c_inst_t *i2c, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = {(reg >> 8) & 0xFF, reg & 0xFF, val};
    int ret = i2c_write_blocking(i2c, VL53L4CX_ADDR, buf, 3, false);
    return ret >= 0;
}

static bool vl53_write_reg16(i2c_inst_t *i2c, uint16_t reg, uint16_t val) {
    uint8_t buf[4] = {(reg >> 8) & 0xFF, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF};
    int ret = i2c_write_blocking(i2c, VL53L4CX_ADDR, buf, 4, false);
    return ret >= 0;
}

static bool vl53_read_reg8(i2c_inst_t *i2c, uint16_t reg, uint8_t *val) {
    uint8_t reg_buf[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    int ret = i2c_write_blocking(i2c, VL53L4CX_ADDR, reg_buf, 2, true);
    if (ret < 0) return false;
    ret = i2c_read_blocking(i2c, VL53L4CX_ADDR, val, 1, false);
    return ret >= 0;
}

static bool vl53_read_reg16(i2c_inst_t *i2c, uint16_t reg, uint16_t *val) {
    uint8_t reg_buf[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    uint8_t data[2];
    int ret = i2c_write_blocking(i2c, VL53L4CX_ADDR, reg_buf, 2, true);
    if (ret < 0) return false;
    ret = i2c_read_blocking(i2c, VL53L4CX_ADDR, data, 2, false);
    if (ret < 0) return false;
    *val = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

bool vl53_init(i2c_inst_t *i2c) {
    uint8_t model_id;
    
    // Wait for device boot (up to 1 second)
    for (int i = 0; i < 100; i++) {
        if (vl53_read_reg8(i2c, VL53L4CX_FIRMWARE_SYSTEM_STATUS, &model_id)) {
            if (model_id == 0x01) {
                break;  // Device is ready
            }
        }
        sleep_ms(10);
    }
    
    // Read Model ID
    if (!vl53_read_reg8(i2c, VL53L4CX_IDENTIFICATION_MODEL_ID, &model_id)) {
        printf("[VL53L4CX] I2C Read Failed\n");
        return false;
    }
    
    printf("[VL53L4CX] Model ID: 0x%02X (Expected 0x%02X)\n", model_id, VL53L4CX_MODEL_ID_VAL);
    
    // Note: VL53L4CX has a complex init sequence handled by ST's API.
    // For basic operation, sensor should work with default settings after boot.
    // Full calibration/configuration requires the official ULD driver.
    
    if (model_id != VL53L4CX_MODEL_ID_VAL) {
        printf("[VL53L4CX] Init WARNING - Model ID mismatch, may be different sensor\n");
    }
    
    printf("[VL53L4CX] Init SUCCESS (Basic Mode)\n");
    return true;
}

void vl53_start_ranging(i2c_inst_t *i2c) {
    // Start continuous ranging
    vl53_write_reg8(i2c, VL53L4CX_SYSTEM_START, 0x40);
}

void vl53_stop_ranging(i2c_inst_t *i2c) {
    vl53_write_reg8(i2c, VL53L4CX_SYSTEM_START, 0x00);
}

bool vl53_data_ready(i2c_inst_t *i2c) {
    uint8_t status;
    if (!vl53_read_reg8(i2c, VL53L4CX_RESULT_RANGE_STATUS, &status)) {
        return false;
    }
    // Check if data is ready (bit 0 of range status indicates new data)
    return (status & 0x01) != 0;
}

void vl53_read_data(i2c_inst_t *i2c, vl53_data_t *data) {
    uint16_t range;
    uint8_t status;
    
    // Read range status
    if (!vl53_read_reg8(i2c, VL53L4CX_RESULT_RANGE_STATUS, &status)) {
        data->range_mm = 0;
        data->status = 0xFF;
        return;
    }
    
    // Read final range in mm
    if (!vl53_read_reg16(i2c, VL53L4CX_RESULT_FINAL_RANGE_MM, &range)) {
        data->range_mm = 0;
        data->status = 0xFF;
        return;
    }
    
    data->range_mm = range;
    data->status = status;
    
    // Clear interrupt (acknowledge reading)
    vl53_write_reg8(i2c, VL53L4CX_SYSTEM_INTERRUPT_CLEAR, 0x01);
}
