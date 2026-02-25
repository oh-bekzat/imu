#include "berryimu.hpp"
#include "i2c_device.hpp"
#include <cmath>

static inline int16_t combine(uint8_t lo, uint8_t hi) {
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

struct BerryIMU::Impl {
    I2CDevice imu;
    I2CDevice mag;
    Impl() : imu("/dev/i2c-1", 0x6b), mag("/dev/i2c-1", 0x1e) {}
};

BerryIMU::BerryIMU() : p(new Impl()) {}
BerryIMU::~BerryIMU() { delete p; }

bool BerryIMU::initialize() {
    if (p->imu.readReg(0x0F) != 0x69) return false;
    if (p->mag.readReg(0x0F) != 0x3D) return false;

    p->imu.writeReg(0x10, 0x40); // accel 104Hz ±2g
    p->imu.writeReg(0x11, 0x40); // gyro  104Hz 245 dps

    p->mag.writeReg(0x20, 0x70);
    p->mag.writeReg(0x21, 0x00);
    p->mag.writeReg(0x22, 0x00);
    p->mag.writeReg(0x23, 0x0C);

    return true;
}

IMUData BerryIMU::read() {
    IMUData d{};

    uint8_t buf[12];
    p->imu.readBytes(0x22, buf, 12);

    int16_t gx = combine(buf[0], buf[1]);
    int16_t gy = combine(buf[2], buf[3]);
    int16_t gz = combine(buf[4], buf[5]);

    int16_t ax = combine(buf[6],  buf[7]);
    int16_t ay = combine(buf[8],  buf[9]);
    int16_t az = combine(buf[10], buf[11]);

    uint8_t mbuf[6];
    p->mag.readBytes(uint8_t(0x28 | 0x80), mbuf, 6);

    int16_t mx = combine(mbuf[0], mbuf[1]);
    int16_t my = combine(mbuf[2], mbuf[3]);
    int16_t mz = combine(mbuf[4], mbuf[5]);

    const float ACCEL_G_PER_LSB  = 0.000061f;               // g/LSB for ±2g
    const float GYRO_DPS_PER_LSB = 0.00875f;                // dps/LSB for 245 dps
    const float DEG2RAD          = (float)M_PI / 180.0f;

    // Madgwick can use accel in g, gyro in rad/s, mag arbitrary units (normalized internally)
    d.ax = ax * ACCEL_G_PER_LSB;
    d.ay = ay * ACCEL_G_PER_LSB;
    d.az = az * ACCEL_G_PER_LSB;

    d.gx = gx * GYRO_DPS_PER_LSB * DEG2RAD;
    d.gy = gy * GYRO_DPS_PER_LSB * DEG2RAD;
    d.gz = gz * GYRO_DPS_PER_LSB * DEG2RAD;

    d.mx = (float)mx;
    d.my = (float)my;
    d.mz = (float)mz;

    return d;
}
