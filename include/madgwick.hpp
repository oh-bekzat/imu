#pragma once
#include <cstdint>

class Madgwick {
public:
    explicit Madgwick(float beta = 0.10f);

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    float q0() const { return q.w; }
    float q1() const { return q.x; }
    float q2() const { return q.y; }
    float q3() const { return q.z; }

private:
    struct Quaternion { float w, x, y, z; } q;
    float beta;
};
