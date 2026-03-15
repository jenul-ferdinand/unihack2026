#include "imu_mag.h"

// Minimal min/max calibration for the ICM-20948 magnetometer.

void imuMagInit(ImuMagCal &cal)
{
    cal.offset[0] = cal.offset[1] = cal.offset[2] = 0.0f;
    cal.scale[0] = cal.scale[1] = cal.scale[2] = 1.0f;
}

static float gMinMag[3] = { 1e9f, 1e9f, 1e9f };
static float gMaxMag[3] = { -1e9f, -1e9f, -1e9f };

void imuMagObserve(ImuMagCal &cal, float mx, float my, float mz)
{
    (void)cal;

    if (mx < gMinMag[0]) gMinMag[0] = mx;
    if (my < gMinMag[1]) gMinMag[1] = my;
    if (mz < gMinMag[2]) gMinMag[2] = mz;

    if (mx > gMaxMag[0]) gMaxMag[0] = mx;
    if (my > gMaxMag[1]) gMaxMag[1] = my;
    if (mz > gMaxMag[2]) gMaxMag[2] = mz;
}

void imuMagFinish(ImuMagCal &cal)
{
    for (int i = 0; i < 3; ++i)
    {
        cal.offset[i] = 0.5f * (gMaxMag[i] + gMinMag[i]);
        const float halfRange = 0.5f * (gMaxMag[i] - gMinMag[i]);
        cal.scale[i] = (halfRange > 1e-3f) ? (1.0f / halfRange) : 1.0f;
    }

    const float avg =
        (cal.scale[0] + cal.scale[1] + cal.scale[2]) / 3.0f;

    if (avg > 1e-6f)
    {
        cal.scale[0] /= avg;
        cal.scale[1] /= avg;
        cal.scale[2] /= avg;
    }
}

void imuMagApply(const ImuMagCal &cal, float rawMx, float rawMy, float rawMz,
                 float &mx, float &my, float &mz)
{
    // Calibration is currently disabled in runtime use. Leaving the formulas in
    // place makes it obvious how to re-enable once the project trusts the
    // collected min/max sweep.
    return;
    mx = (rawMx - cal.offset[0]) * cal.scale[0];
    my = (rawMy - cal.offset[1]) * cal.scale[1];
    mz = (rawMz - cal.offset[2]) * cal.scale[2];
}
