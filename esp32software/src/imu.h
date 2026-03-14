#pragma once

bool imu_begin();
void imu_update();

void imu_getRawAccel(float &ax, float &ay, float &az);
void imu_getRawGyro(float &gx, float &gy, float &gz);
void imu_getRawMag(float &mx, float &my, float &mz);

void imu_getLinearAccel(float &ax, float &ay, float &az);
void imu_getLinearAccelWorld(float &ax, float &ay, float &az);

void imu_getQuat(float &q0, float &q1, float &q2, float &q3);
void imu_getEuler(float &roll, float &pitch, float &yaw);

void imu_getPosition(float &x, float &y, float &z);
void imu_getVelocity(float &x, float &y, float &z);
void imu_resetPosition();

void imu_printDebug();

void imu_setStationary(bool still);
void imu_zeroVelocity();
void imu_resetPosition();