#pragma once

struct Quaternion {
    float w, x, y, z;
};

class AHRS {
public:
    AHRS(float beta = 0.1f);
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    Quaternion getQuaternion() const;

private:
    float beta;
    Quaternion q;
};
