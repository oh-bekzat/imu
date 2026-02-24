#include "i2c_device.hpp"
#include "lis3mdl.hpp"
#include "madgwick.hpp"

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <cstdint>

static inline int16_t combine(uint8_t lo, uint8_t hi) {
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

int main() {
    try {
        I2CDevice imu("/dev/i2c-1", 0x6b);
        LIS3MDL mag;
        Madgwick filter(0.10f);

        uint8_t who = imu.readReg(0x0f);
        if (who != 0x69) {
            std::cerr << "LSM6DS33 not detected (WHO_AM_I mismatch)!\n";
            return 1;
        }

        // Accel: 104 Hz, ±2g
        imu.writeReg(0x10, 0x40);
        // Gyro: 104 Hz, 245 dps
        imu.writeReg(0x11, 0x40);

        if (!mag.begin()) {
            std::cerr << "Magnetometer init failed!\n";
            return 1;
        }

        std::cout << "All sensors initialized.\n";

        // LSM6DS33 sensitivities for our config
        const float ACCEL_G_PER_LSB   = 0.000061f;                 // g/LSB
        const float GYRO_DPS_PER_LSB  = 0.00875f;                  // dps/LSB
        const float DEG2RAD           = (float)M_PI / 180.0f;      // rad/deg

        // 100 Hz loop
        const float dt = 0.01f;
        const useconds_t sleep_us = 10000;

        while (true) {
            uint8_t buffer[12];
            imu.readBytes(0x22, buffer, 12);

            int16_t gx_raw = combine(buffer[0], buffer[1]);
            int16_t gy_raw = combine(buffer[2], buffer[3]);
            int16_t gz_raw = combine(buffer[4], buffer[5]);

            int16_t ax_raw = combine(buffer[6], buffer[7]);
            int16_t ay_raw = combine(buffer[8], buffer[9]);
            int16_t az_raw = combine(buffer[10], buffer[11]);

            int16_t mx_raw, my_raw, mz_raw;
            mag.readRaw(mx_raw, my_raw, mz_raw);

            // Convert accel to g (unitless), gyro to rad/s
            float ax = ax_raw * ACCEL_G_PER_LSB;
            float ay = ay_raw * ACCEL_G_PER_LSB;
            float az = az_raw * ACCEL_G_PER_LSB;

            float gx = gx_raw * GYRO_DPS_PER_LSB * DEG2RAD;
            float gy = gy_raw * GYRO_DPS_PER_LSB * DEG2RAD;
            float gz = gz_raw * GYRO_DPS_PER_LSB * DEG2RAD;

            // Magnetometer: raw is fine (Madgwick normalizes it)
            float mx = (float)mx_raw;
            float my = (float)my_raw;
            float mz = (float)mz_raw;

            filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

            // Optional normalization check
            float n = std::sqrt(filter.q0()*filter.q0() + filter.q1()*filter.q1() +
                                filter.q2()*filter.q2() + filter.q3()*filter.q3());

            std::cout << "norm=" << n << " Q: "
                      << filter.q0() << " "
                      << filter.q1() << " "
                      << filter.q2() << " "
                      << filter.q3() << "\n";

            usleep(sleep_us);
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
