#include "imu_math.h"
#include <math.h>
#include <Arduino.h>

static constexpr float G_MSS = 9.80665f;

void imuMathRotateVectorByQuat(const float q[4], const float v[3], float out[3])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float tx = 2.0f * (qy * v[2] - qz * v[1]);
    const float ty = 2.0f * (qz * v[0] - qx * v[2]);
    const float tz = 2.0f * (qx * v[1] - qy * v[0]);

    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

void imuMathComputeGravityBodyFromQuat(const float q[4], float gBody[3])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float gx = 2.0f * (qx * qz - qw * qy);
    const float gy = 2.0f * (qw * qx + qy * qz);
    const float gz = qw * qw - qx * qx - qy * qy + qz * qz;

    gBody[0] = gx * G_MSS;
    gBody[1] = gy * G_MSS;
    gBody[2] = gz * G_MSS;
}

void imuMathQuatToEulerDeg(const float q[4], float &rollDeg, float &pitchDeg, float &yawDeg)
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    rollDeg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

    const float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        pitchDeg = copysignf(90.0f, sinp);
    else
        pitchDeg = asinf(sinp) * 180.0f / PI;

    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yawDeg = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

float imuMathNorm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void imuMathApplyDeadband(float v[3], float threshold)
{
    for (int i = 0; i < 3; ++i)
    {
        if (fabsf(v[i]) < threshold)
            v[i] = 0.0f;
    }
}