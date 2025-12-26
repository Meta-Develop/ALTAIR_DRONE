#ifndef VL53L4CX_H
#define VL53L4CX_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define VL53L4CX_ADDR 0x29
#define VL53L4CX_ID   0xEB

class VL53L4CX {
public:
    VL53L4CX(i2c_inst_t* i2c, uint8_t addr = VL53L4CX_ADDR);
    bool init();
    bool read_distance(uint16_t* distance_mm);

private:
    i2c_inst_t* _i2c;
    uint8_t _addr;

    void write_reg(uint16_t reg, uint8_t value);
    void write_reg16(uint16_t reg, uint16_t value);
    void write_reg32(uint16_t reg, uint32_t value);
    uint8_t read_reg(uint16_t reg);
    void read_regs(uint16_t reg, uint8_t* buf, int len);
    
    void wait_for_boot();
};

#endif
