#include "ahrs.hpp"
#include <cmath>

static inline float invSqrt(float x) {
    return 1.0f / std::sqrt(x);
}

AHRS::AHRS(float beta_) : beta(beta_) {
    q = {1.0f, 0.0f, 0.0f, 0.0f};
}

Quaternion AHRS::getQuaternion() const {
    return q;
}

void AHRS::update(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz,
                  float dt)
{
    float q1=q.w, q2=q.x, q3=q.y, q4=q.z;

    float norm = ax*ax + ay*ay + az*az;
    if (norm <= 0.0f) return;
    norm = invSqrt(norm);
    ax*=norm; ay*=norm; az*=norm;

    norm = mx*mx + my*my + mz*mz;
    if (norm <= 0.0f) return;
    norm = invSqrt(norm);
    mx*=norm; my*=norm; mz*=norm;

    float _2q1mx = 2.0f*q1*mx;
    float _2q1my = 2.0f*q1*my;
    float _2q1mz = 2.0f*q1*mz;
    float _2q2mx = 2.0f*q2*mx;

    float _2q1 = 2.0f*q1;
    float _2q2 = 2.0f*q2;
    float _2q3 = 2.0f*q3;
    float _2q4 = 2.0f*q4;

    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q1q4 = q1*q4;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q2q4 = q2*q4;
    float q3q3 = q3*q3;
    float q3q4 = q3*q4;
    float q4q4 = q4*q4;

    float hx = mx*q1q1 - _2q1my*q4 + _2q1mz*q3 + mx*q2q2 + _2q2*my*q3 + _2q2*mz*q4 - mx*q3q3 - mx*q4q4;
    float hy = _2q1mx*q4 + my*q1q1 - _2q1mz*q2 + _2q2mx*q3 - my*q2q2 + my*q3q3 + _2q3*mz*q4 - my*q4q4;
    float _2bx = std::sqrt(hx*hx + hy*hy);
    float _2bz = -_2q1mx*q3 + _2q1my*q2 + mz*q1q1 + _2q2mx*q4 - mz*q2q2 + _2q3*my*q4 - mz*q3q3 + mz*q4q4;
    float _4bx = 2.0f*_2bx;
    float _4bz = 2.0f*_2bz;

    float s1 = -_2q3*(2.0f*(q2q4 - q1q3) - ax) + _2q2*(2.0f*(q1q2 + q3q4) - ay)
             - _2bz*q3*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (-_2bx*q4 + _2bz*q2)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + _2bx*q3*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s2 =  _2q4*(2.0f*(q2q4 - q1q3) - ax) + _2q1*(2.0f*(q1q2 + q3q4) - ay)
             - 4.0f*q2*(1.0f - 2.0f*(q2q2 + q3q3) - az)
             + _2bz*q4*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (_2bx*q3 + _2bz*q1)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + (_2bx*q4 - _4bz*q2)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s3 = -_2q1*(2.0f*(q2q4 - q1q3) - ax) + _2q4*(2.0f*(q1q2 + q3q4) - ay)
             - 4.0f*q3*(1.0f - 2.0f*(q2q2 + q3q3) - az)
             + (-_4bx*q3 - _2bz*q1)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (_2bx*q2 + _2bz*q4)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + (_2bx*q1 - _4bz*q3)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s4 =  _2q2*(2.0f*(q2q4 - q1q3) - ax) + _2q3*(2.0f*(q1q2 + q3q4) - ay)
             + (-_4bx*q4 + _2bz*q2)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (-_2bx*q1 + _2bz*q3)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + _2bx*q2*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    norm = invSqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    s1*=norm; s2*=norm; s3*=norm; s4*=norm;

    float qDot1 = 0.5f*(-q2*gx - q3*gy - q4*gz) - beta*s1;
    float qDot2 = 0.5f*( q1*gx + q3*gz - q4*gy) - beta*s2;
    float qDot3 = 0.5f*( q1*gy - q2*gz + q4*gx) - beta*s3;
    float qDot4 = 0.5f*( q1*gz + q2*gy - q3*gx) - beta*s4;

    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    norm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q = {q1*norm, q2*norm, q3*norm, q4*norm};
}
