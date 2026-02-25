#pragma once
#include <cstdint>
#include <string>

class I2CDevice {
public:
    I2CDevice(const std::string& device, uint8_t address);
    ~I2CDevice();

    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);
    void readBytes(uint8_t reg, uint8_t* buffer, size_t length);

private:
    int fd;
};
