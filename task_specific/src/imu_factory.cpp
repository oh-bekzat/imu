#include "berryimu.hpp"

IMU* createIMU() {
    return new BerryIMU();
}
