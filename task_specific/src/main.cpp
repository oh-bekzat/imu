#include "imu.hpp"
#include "ahrs.hpp"
#include <iostream>
#include <chrono>
#include <thread>

extern IMU* createIMU(); // we'll define this below

int main() {
    IMU* imu = createIMU();

    if (!imu->initialize()) {
        std::cout << "IMU init failed\n";
        return -1;
    }

    AHRS filter;

    auto last = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = now - last;
        last = now;

        IMUData data = imu->read();

        filter.update(data.gx, data.gy, data.gz,
                      data.ax, data.ay, data.az,
                      data.mx, data.my, data.mz,
                      dt.count());

        Quaternion q = filter.getQuaternion();

        std::cout << q.w << " "
                  << q.x << " "
                  << q.y << " "
                  << q.z << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
