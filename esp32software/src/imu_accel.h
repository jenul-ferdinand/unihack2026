#pragma once

#ifndef CLAMP_RADIUS_M
#define CLAMP_RADIUS_M 2.0f
#endif

#include "debug_config.h"

#define IMU_DEBUG (DEBUG_SERIAL_ENABLE && DEBUG_IMU_ACCEL)
#define GYRO_DEBUG (DEBUG_SERIAL_ENABLE && DEBUG_GYRO_CLAMP)
#define GYRO_NOISE_THRESH_DPS 30.0f

// Translational-motion helper used underneath imu.cpp.
//
// This layer removes gravity, rotates body acceleration into world space,
// integrates velocity/position during obvious motion bursts, and aggressively
// damps the estimate when motion confidence is low.

struct ImuAccelCal
{
    float gyroBiasDps[3];
    float accelBiasMss[3];
};

struct ImuMotionState
{
    float linBody[3];
    float linWorld[3];
    float velWorld[3];
    float posWorld[3];

    // Hysteretic gate that decides whether translational integration is allowed.
    bool motionBurst;

    float travelMeters;   // total path length since reset
    float pendingMeters;  // unsent travelled distance
};

bool imuAccelConsumeMeter(ImuMotionState &motion);

void imuAccelInit(ImuAccelCal &cal, ImuMotionState &motion);
void imuAccelCalibrateStill(ImuAccelCal &cal,
                            float sumGx, float sumGy, float sumGz,
                            float sumAx, float sumAy, float sumAz,
                            int samples);

void imuAccelProcess(ImuMotionState &motion,
                     const float quat[4],
                     float rawAx, float rawAy, float rawAz,
                     float rawGx, float rawGy, float rawGz,
                     float dt,
                     bool stationaryHold);

// Maintenance helpers used by the sketch's hold logic.
void imuAccelZeroVelocity(ImuMotionState &motion);
void imuAccelResetPosition(ImuMotionState &motion);
