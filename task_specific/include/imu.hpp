#pragma once

struct IMUData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
};

class IMU {
public:
    virtual ~IMU() = default;
    virtual bool initialize() = 0;
    virtual IMUData read() = 0;
};
