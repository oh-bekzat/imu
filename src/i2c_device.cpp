#include "i2c_device.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

I2CDevice::I2CDevice(const std::string& device, uint8_t address) {
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0)
        throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd, I2C_SLAVE, address) < 0)
        throw std::runtime_error("Failed to select I2C device");
}

I2CDevice::~I2CDevice() {
    close(fd);
}

uint8_t I2CDevice::readReg(uint8_t reg) {
    write(fd, &reg, 1);
    uint8_t data;
    read(fd, &data, 1);
    return data;
}

void I2CDevice::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    write(fd, buffer, 2);
}

void I2CDevice::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    write(fd, &reg, 1);
    read(fd, buffer, length);
}
