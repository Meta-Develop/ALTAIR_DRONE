#ifndef BMP388_H
#define BMP388_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Registers
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_ERR          0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA_0       0x04
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB_DATA   0x31
#define BMP388_REG_CMD          0x7E

#define BMP388_CHIP_ID          0x50
#define BMP388_ADDR             0x77 // Default Qwiic

struct BMP388_CalibData {
    uint16_t T1, T2;
    int8_t   T3;
    int16_t  P1, P2;
    int8_t   P3, P4;
    uint16_t P5, P6;
    int8_t   P7, P8;
    int16_t  P9;
    int8_t   P10, P11;
};

class BMP388 {
public:
    BMP388(i2c_inst_t* i2c, uint8_t addr = BMP388_ADDR);
    bool init();
    bool read_data(float* pressure, float* temperature); // Returns Pa and DegC

private:
    i2c_inst_t* _i2c;
    uint8_t _addr;
    BMP388_CalibData _calib;

    void write_reg(uint8_t reg, uint8_t value);
    uint8_t read_reg(uint8_t reg);
    void read_regs(uint8_t reg, uint8_t* buf, int len);
    
    void read_calibration();
    float compensate_temperature(uint32_t uncomp_temp);
    float compensate_pressure(uint32_t uncomp_press, float compensated_temp);
};

#endif
