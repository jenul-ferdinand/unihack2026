#include "time_sync.h"
#include <math.h>

// A deliberately lightweight synchronizer: enough to align timestamps between
// two ESP32 nodes, but not intended to behave like a full NTP/PTP stack.

static double clampDouble(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void timeSyncBegin(TimeSyncState &ts, bool isMaster)
{
    ts.isMaster = isMaster;
    ts.locked = isMaster;

    ts.rate = 1.0;
    ts.offset = 0.0;

    ts.lastLocalUs = 0;
    ts.lastMasterUs = 0;
    ts.havePrev = false;
    ts.sampleCount = 0;
}

void timeSyncObserveMaster(TimeSyncState &ts, uint32_t localRxUs, uint32_t masterUs)
{
    if (ts.isMaster)
        return;

    if (!ts.havePrev)
    {
        ts.rate = 1.0;
        ts.offset = (double)masterUs - (double)localRxUs;
        ts.lastLocalUs = localRxUs;
        ts.lastMasterUs = masterUs;
        ts.havePrev = true;
        ts.sampleCount = 1;
        ts.locked = false;
        return;
    }

    uint32_t dLocal = localRxUs - ts.lastLocalUs;
    uint32_t dMaster = masterUs - ts.lastMasterUs;

    if (dLocal > 1000 && dMaster > 1000)
    {
        double observedRate = (double)dMaster / (double)dLocal;

        // Keep this sane. Real crystal drift should be tiny and slow.
        observedRate = clampDouble(observedRate, 0.9995, 1.0005);

        // Slow low-pass on drift estimate.
        const double rateAlpha = 0.02;
        ts.rate = (1.0 - rateAlpha) * ts.rate + rateAlpha * observedRate;
    }

    double observedOffset = (double)masterUs - ts.rate * (double)localRxUs;

    // Slightly faster low-pass on offset because transport delay shifts offset
    // more readily than it shifts long-term frequency error.
    const double offsetAlpha = 0.05;
    ts.offset = (1.0 - offsetAlpha) * ts.offset + offsetAlpha * observedOffset;

    ts.lastLocalUs = localRxUs;
    ts.lastMasterUs = masterUs;

    if (ts.sampleCount < 65535)
        ts.sampleCount++;

    if (ts.sampleCount >= 20)
        ts.locked = true;
}

uint32_t timeSyncNowUs(const TimeSyncState &ts, uint32_t localUs)
{
    if (ts.isMaster)
        return localUs;

    double corrected = ts.rate * (double)localUs + ts.offset;

    if (corrected < 0.0)
        corrected = 0.0;

    return (uint32_t)(corrected + 0.5);
}

bool timeSyncLocked(const TimeSyncState &ts)
{
    return ts.locked;
}

float timeSyncRatePpm(const TimeSyncState &ts)
{
    return (float)((ts.rate - 1.0) * 1000000.0);
}

int32_t timeSyncOffsetUs(const TimeSyncState &ts, uint32_t localUs)
{
    double corrected = ts.rate * (double)localUs + ts.offset;
    return (int32_t)(corrected - (double)localUs);
}
