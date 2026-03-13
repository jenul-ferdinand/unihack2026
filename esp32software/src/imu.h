#pragma once

#include <stdint.h>
#include <Arduino.h>

bool imu_begin();                     // initialize IMU, returns true on success
void imu_update();                    // read IMU and update internal filter
void imu_getLinearAccel(float &ax, float &ay, float &az); // gravity removed accel m/s^2
void imu_getQuat(float &q0, float &q1, float &q2, float &q3); // current quaternion