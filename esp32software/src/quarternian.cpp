#include <cmath>
static inline void imuMathQuatToEulerDeg(const float q[4], float &rollDeg, float &pitchDeg, float &yawDeg)
{
    // Assumes q = [w, x, y, z]
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    // Roll (x-axis rotation)
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    rollDeg = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    // Pitch (y-axis rotation)
    const float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        pitchDeg = copysignf(90.0f, sinp);
    else
        pitchDeg = asinf(sinp) * 57.2957795f;

    // Yaw (z-axis rotation)
    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yawDeg = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
}