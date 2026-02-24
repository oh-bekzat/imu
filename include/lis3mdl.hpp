#pragma once
#include "i2c_device.hpp"
#include <cstdint>

class LIS3MDL {
public:
    explicit LIS3MDL(const char* i2c_dev = "/dev/i2c-1", uint8_t addr = 0x1e);

    bool begin();                 // config + WHO_AM_I check
    void readRaw(int16_t& mx, int16_t& my, int16_t& mz);

private:
    I2CDevice dev;
    static int16_t combine(uint8_t lo, uint8_t hi);
};
