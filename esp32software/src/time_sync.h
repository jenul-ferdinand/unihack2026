#pragma once
#include <Arduino.h>

struct TimeSyncState
{
    bool     isMaster = false;
    bool     locked = false;

    double   rate = 1.0;     // master_us per local_us
    double   offset = 0.0;   // master_us - rate * local_us

    uint32_t lastLocalUs = 0;
    uint32_t lastMasterUs = 0;
    bool     havePrev = false;

    uint16_t sampleCount = 0;
};

void timeSyncBegin(TimeSyncState &ts, bool isMaster);
void timeSyncObserveMaster(TimeSyncState &ts, uint32_t localRxUs, uint32_t masterUs);
uint32_t timeSyncNowUs(const TimeSyncState &ts, uint32_t localUs);
bool timeSyncLocked(const TimeSyncState &ts);
float timeSyncRatePpm(const TimeSyncState &ts);
int32_t timeSyncOffsetUs(const TimeSyncState &ts, uint32_t localUs);