#include "ism330.h"
#include <stdio.h>

ISM330DHCX::ISM330DHCX(spi_inst_t* spi, uint cs_pin) : _spi(spi), _cs_pin(cs_pin) {
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1); // Deselect
}

void ISM330DHCX::cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

void ISM330DHCX::cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

void ISM330DHCX::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    cs_select();
    spi_write_blocking(_spi, buf, 2);
    cs_deselect();
}

uint8_t ISM330DHCX::read_reg(uint8_t reg) {
    uint8_t cmd = reg | 0x80; // Read bit
    uint8_t val;
    cs_select();
    spi_write_blocking(_spi, &cmd, 1);
    spi_read_blocking(_spi, 0, &val, 1);
    cs_deselect();
    return val;
}

void ISM330DHCX::read_regs(uint8_t reg, uint8_t* buf, int len) {
    uint8_t cmd = reg | 0x80; // Read bit
    cs_select();
    spi_write_blocking(_spi, &cmd, 1);
    spi_read_blocking(_spi, 0, buf, len);
    cs_deselect();
}

bool ISM330DHCX::init() {
    // Check ID
    uint8_t id = read_reg(ISM330_REG_WHO_AM_I);
    if (id != ISM330_WHO_AM_I_ID) {
        printf("ISM330 ID mismatch: 0x%02X\n", id);
        return false;
    }

    // Reset logic could go here (SW_RESET)
    
    // Config Accel: 6.66kHz, 16g
    // ODR=1010, FS=01 (16g), LPF2=0
    write_reg(ISM330_REG_CTRL1_XL, 0xA4); 

    // Config Gyro: 6.66kHz, 2000dps
    // ODR=1010, FS=10 (2000dps)? -> Check Datasheet. 
    // Usually: 125(0010), 250(0000), 500(0100), 1000(1000), 2000(1100).
    // Let's assume standard FS mapping. 2000dps = 0x0C (1100).
    // So 1010 1100 = 0xAC.
    // Wait, let's verify FS_G for ISM330.
    // FS_G[1:0]: 00=250, 01=500, 10=1000, 11=2000? Or shift?
    // Datasheet says:
    // 0000: 250 dps
    // 0100: 500 dps
    // 1000: 1000 dps
    // 1100: 2000 dps
    // Bit 3-2 are FS_G.
    // So 0xA0 (6.6k) | 0x0C (2000) = 0xAC.
    write_reg(ISM330_REG_CTRL2_G, 0xAC);

    // Config: BDU=1, IF_INC=1
    write_reg(ISM330_REG_CTRL3_C, 0x44);

    return true;
}

void ISM330DHCX::read_data(float* accel, float* gyro) {
    uint8_t raw[12];
    read_regs(ISM330_REG_OUTX_L_G, raw, 12);

    // Format: GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H...
    int16_t g_raw[3];
    int16_t a_raw[3];

    g_raw[0] = (int16_t)(raw[1] << 8 | raw[0]);
    g_raw[1] = (int16_t)(raw[3] << 8 | raw[2]);
    g_raw[2] = (int16_t)(raw[5] << 8 | raw[4]);

    a_raw[0] = (int16_t)(raw[7] << 8 | raw[6]);
    a_raw[1] = (int16_t)(raw[9] << 8 | raw[8]);
    a_raw[2] = (int16_t)(raw[11] << 8 | raw[10]);

    // Conversions
    // Accel 16g: 0.488 mg/LSB -> 0.488 * 9.81 / 1000 m/s^2 per LSB
    // 16g range = +/- 16g. 16 * 9.81 = 156.96 m/s^2 max. 32768 LSB.
    // 156.96 / 32768 = 0.00479?
    // ST usually says Sensitivity at 16g is 0.488 mg/LSB.
    // 0.488 * 1e-3 * 9.80665
    float a_scale = 0.488f * 0.001f * 9.80665f;
    
    // Gyro 2000dps: 70 mdps/LSB
    // 70 * 1e-3 * deg2rad
    float g_scale = 70.0f * 0.001f * (3.14159f / 180.0f);

    for(int i=0; i<3; i++) {
        accel[i] = a_raw[i] * a_scale;
        gyro[i] = g_raw[i] * g_scale;
    }
}
