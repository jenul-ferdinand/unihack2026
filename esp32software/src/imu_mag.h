#pragma once

// Magnetometer calibration state.
//
// The code tracks axis minima/maxima during a startup sweep, derives hard-iron
// offsets plus a simple scale normalization, and can later apply that
// calibration to raw magnetometer samples.

struct ImuMagCal
{
    float offset[3];
    float scale[3];
};

void imuMagInit(ImuMagCal &cal);
void imuMagObserve(ImuMagCal &cal, float mx, float my, float mz);
void imuMagFinish(ImuMagCal &cal);
void imuMagApply(const ImuMagCal &cal, float rawMx, float rawMy, float rawMz,
                 float &mx, float &my, float &mz);
