#pragma once

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
};

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