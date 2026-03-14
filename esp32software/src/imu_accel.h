#pragma once

#ifndef CLAMP_RADIUS_M
#define CLAMP_RADIUS_M 2.0f
#endif

#define IMU_DEBUG 1

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

void imuAccelZeroVelocity(ImuMotionState &motion);
void imuAccelResetPosition(ImuMotionState &motion);