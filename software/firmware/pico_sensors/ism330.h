#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

// Registers
#define ISM330_REG_WHO_AM_I    0x0F
#define ISM330_REG_CTRL1_XL    0x10
#define ISM330_REG_CTRL2_G     0x11
#define ISM330_REG_CTRL3_C     0x12
#define ISM330_REG_STATUS      0x1E
#define ISM330_REG_OUTX_L_G    0x22

#define ISM330_WHO_AM_I_ID     0x6B

class ISM330DHCX {
public:
    ISM330DHCX(spi_inst_t* spi, uint cs_pin);
    bool init();
    void read_data(float* accel, float* gyro);
    uint8_t read_reg(uint8_t reg);

private:
    spi_inst_t* _spi;
    uint _cs_pin;

    void write_reg(uint8_t reg, uint8_t value);
    // uint8_t read_reg(uint8_t reg); // Moved to public
    void read_regs(uint8_t reg, uint8_t* buf, int len);
    
    void cs_select();
    void cs_deselect();
};

#endif
