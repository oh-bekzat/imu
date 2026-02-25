#pragma once
#include "imu.hpp"

class BerryIMU : public IMU {
public:
    BerryIMU();
    ~BerryIMU();

    bool initialize() override;
    IMUData read() override;

private:
    struct Impl;
    Impl* p;
};
