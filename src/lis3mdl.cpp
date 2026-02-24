#include "lis3mdl.hpp"
#include <iostream>

static constexpr uint8_t REG_WHO_AM_I = 0x0F;
static constexpr uint8_t WHO_AM_I_VAL = 0x3D;

// Control registers
static constexpr uint8_t REG_CTRL_REG1 = 0x20;
static constexpr uint8_t REG_CTRL_REG2 = 0x21;
static constexpr uint8_t REG_CTRL_REG3 = 0x22;
static constexpr uint8_t REG_CTRL_REG4 = 0x23;

// Output registers (LIS3MDL is little-endian, X/Y/Z, 6 bytes)
static constexpr uint8_t REG_OUT_X_L = 0x28;

int16_t LIS3MDL::combine(uint8_t lo, uint8_t hi) {
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

LIS3MDL::LIS3MDL(const char* i2c_dev, uint8_t addr)
    : dev(i2c_dev, addr) {}

bool LIS3MDL::begin() {
    uint8_t who = dev.readReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        std::cerr << "LIS3MDL WHO_AM_I unexpected: 0x"
                  << std::hex << (int)who << std::dec << std::endl;
        return false;
    }

    // Recommended simple configuration:
    // CTRL_REG1: Temp off, Ultra-high performance XY, ODR=10 Hz
    dev.writeReg(REG_CTRL_REG1, 0b01110000); // 0x70

    // CTRL_REG2: +/-4 gauss full scale (00)
    dev.writeReg(REG_CTRL_REG2, 0x00);

    // CTRL_REG3: Continuous-conversion mode
    dev.writeReg(REG_CTRL_REG3, 0x00);

    // CTRL_REG4: Ultra-high performance Z
    dev.writeReg(REG_CTRL_REG4, 0b00001100); // 0x0C

    return true;
}

void LIS3MDL::readRaw(int16_t& mx, int16_t& my, int16_t& mz) {
    uint8_t buf[6];
    // Auto-increment: set MSB of register address
    dev.readBytes(REG_OUT_X_L | 0x80, buf, 6);

    mx = combine(buf[0], buf[1]);
    my = combine(buf[2], buf[3]);
    mz = combine(buf[4], buf[5]);
}
