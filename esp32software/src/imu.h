#pragma once

// High-level IMU facade used by the sketch.
//
// This module owns sensor bring-up, 10 second still/magnetometer calibration,
// Madgwick orientation fusion, and the basic integrated motion estimate exposed
// to the rest of the firmware.

bool imu_begin();
void imu_update();

// Raw calibrated sensor readings from the latest IMU sample.
void imu_getRawAccel(float &ax, float &ay, float &az);
void imu_getRawGyro(float &gx, float &gy, float &gz);
void imu_getRawMag(float &mx, float &my, float &mz);

// Gravity-removed acceleration in body/world frames.
void imu_getLinearAccel(float &ax, float &ay, float &az);
void imu_getLinearAccelWorld(float &ax, float &ay, float &az);

// Current fused orientation.
void imu_getQuat(float &q0, float &q1, float &q2, float &q3);
void imu_getEuler(float &roll, float &pitch, float &yaw);

// Integrated world-space motion estimate.
void imu_getPosition(float &x, float &y, float &z);
void imu_getVelocity(float &x, float &y, float &z);
void imu_resetPosition();

// Debug / maintenance helpers.
void imu_printDebug();
void imu_zeroGyroRate();

// Stationary hold tells the module to stop integrating translational motion.
void imu_setStationary(bool still);
void imu_zeroVelocity();
