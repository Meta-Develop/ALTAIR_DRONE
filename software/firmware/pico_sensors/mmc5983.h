#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

// Registers
#define MMC5983_REG_X_OUT_0     0x00
#define MMC5983_REG_X_OUT_1     0x01
#define MMC5983_REG_Y_OUT_0     0x02
#define MMC5983_REG_Y_OUT_1     0x03
#define MMC5983_REG_Z_OUT_0     0x04
#define MMC5983_REG_Z_OUT_1     0x05
#define MMC5983_REG_STATUS      0x06
#define MMC5983_REG_INT_CTRL_0  0x09
#define MMC5983_REG_INT_CTRL_1  0x0A
#define MMC5983_REG_INT_CTRL_2  0x0B
#define MMC5983_REG_PRODUCT_ID  0x2F

#define MMC5983_PRODUCT_ID      0x30

class MMC5983MA {
public:
    MMC5983MA(spi_inst_t* spi, uint cs_pin);
    bool init();
    void read_data(float* mag); // Returns x,y,z in Gauss

private:
    spi_inst_t* _spi;
    uint _cs_pin;

    void write_reg(uint8_t reg, uint8_t value);
    uint8_t read_reg(uint8_t reg);
    void read_regs(uint8_t reg, uint8_t* buf, int len);
    
    void cs_select();
    void cs_deselect();
};

#endif
