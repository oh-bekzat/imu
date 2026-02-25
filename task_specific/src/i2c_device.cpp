#include "i2c_device.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

I2CDevice::I2CDevice(const std::string& device, uint8_t address) {
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd, I2C_SLAVE, address) < 0)
        throw std::runtime_error("Failed to set I2C address");
}

I2CDevice::~I2CDevice() {
    if (fd >= 0) close(fd);
}

uint8_t I2CDevice::readReg(uint8_t reg) {
    if (write(fd, &reg, 1) != 1)
        throw std::runtime_error("I2C write failed");

    uint8_t value{};
    if (read(fd, &value, 1) != 1)
        throw std::runtime_error("I2C read failed");

    return value;
}

void I2CDevice::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(fd, buf, 2) != 2)
        throw std::runtime_error("I2C write failed");
}

void I2CDevice::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(fd, &reg, 1) != 1)
        throw std::runtime_error("I2C write failed");

    if (read(fd, buffer, length) != (ssize_t)length)
        throw std::runtime_error("I2C read failed");
}
