#pragma once

// Small quaternion/vector helpers shared by the IMU processing code.

void imuMathRotateVectorByQuat(const float q[4], const float v[3], float out[3]);
void imuMathComputeGravityBodyFromQuat(const float q[4], float gBody[3]);
void imuMathQuatToEulerDeg(const float q[4], float &rollDeg, float &pitchDeg, float &yawDeg);
float imuMathNorm3(float x, float y, float z);
void imuMathApplyDeadband(float v[3], float threshold);
