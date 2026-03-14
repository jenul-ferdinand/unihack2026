#include "dead_reckoning.h"

#include <math.h>

static constexpr float DEAD_RECKON_ACC_THRESH_MS2 = 0.35f;
static constexpr float DEAD_RECKON_SPEED_DAMP = 0.82f;
static constexpr float DEAD_RECKON_MIN_SPEED_MPS = 0.03f;
static constexpr float DEAD_RECKON_MAX_SPEED_MPS = 4.0f;
static constexpr float PEER_CLOSE_DISTANCE_M = 1.0f;
static constexpr float GYRO_HEADING_GAIN = 4.0f;

struct FacingReference
{
    const char *name;
    float rollDeg;
    float headingDeg;
};

static constexpr FacingReference FACING_REFS[] = {
    {"forward", 1.85f, 0.0f},
    {"front_right", -7.94f, -45.0f},
    {"left", -17.73f, 90.0f},
    {"back_left", -32.13f, 135.0f},
    {"backward", -46.53f, 180.0f},
    {"right", 32.50f, -90.0f},
    {"right", -69.57f, -90.0f},
};

static float vecNorm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float degToRad(float deg)
{
    return deg * PI / 180.0f;
}

static float wrapAngleDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

static void rotate2D(float x, float y, float yawDeg, float &outX, float &outY)
{
    const float r = degToRad(yawDeg);
    const float c = cosf(r);
    const float s = sinf(r);
    outX = c * x - s * y;
    outY = s * x + c * y;
}

static float angleDistanceDeg(float a, float b)
{
    return fabsf(wrapAngleDeg(a - b));
}

static float quantizeToCardinalDeg(float headingDeg)
{
    const float wrapped = wrapAngleDeg(headingDeg);
    const float snapped = roundf(wrapped / 90.0f) * 90.0f;
    return wrapAngleDeg(snapped);
}

static const FacingReference &findClosestFacing(float rollDeg)
{
    const FacingReference *best = &FACING_REFS[0];
    float bestDistance = angleDistanceDeg(rollDeg, best->rollDeg);

    for (unsigned int i = 1; i < sizeof(FACING_REFS) / sizeof(FACING_REFS[0]); ++i)
    {
        const float distance = angleDistanceDeg(rollDeg, FACING_REFS[i].rollDeg);
        if (distance < bestDistance)
        {
            best = &FACING_REFS[i];
            bestDistance = distance;
        }
    }

    return *best;
}

static uint8_t directionCodeFromBearing(float snappedBearingDeg)
{
    if (snappedBearingDeg == 0.0f)
        return LINK_PEER_FORWARD;
    if (snappedBearingDeg == 90.0f)
        return LINK_PEER_RIGHT;
    if (snappedBearingDeg == -90.0f)
        return LINK_PEER_LEFT;
    return LINK_PEER_BACK;
}

static float directionOffsetDeg(uint8_t directionCode)
{
    switch (directionCode)
    {
        case LINK_PEER_FORWARD:
            return 0.0f;
        case LINK_PEER_RIGHT:
            return 90.0f;
        case LINK_PEER_LEFT:
            return -90.0f;
        case LINK_PEER_BACK:
            return 180.0f;
        default:
            return 0.0f;
    }
}

static uint8_t confidenceFromBearing(float bearingLocalDeg)
{
    const float snappedBearing = quantizeToCardinalDeg(bearingLocalDeg);
    const float error = angleDistanceDeg(bearingLocalDeg, snappedBearing);

    if (error <= 10.0f)
        return 3;
    if (error <= 22.5f)
        return 2;
    if (error <= 45.0f)
        return 1;
    return 0;
}

void deadReckoningInit(DeadReckoningState &state, uint32_t nowUs)
{
    state = DeadReckoningState{};
    state.lastUpdateUs = nowUs;
}

void deadReckoningUpdate(DeadReckoningState &state, const DeadReckoningInput &input, uint32_t nowUs)
{
    float dt = 0.02f;
    if (state.lastUpdateUs != 0)
    {
        dt = (nowUs - state.lastUpdateUs) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.02f;
    }
    state.lastUpdateUs = nowUs;

    state.accelMss = vecNorm3(input.linAccelBodyX, input.linAccelBodyY, input.linAccelBodyZ);
    state.stepMeters = 0.0f;
    state.gyroHeadingDeg = wrapAngleDeg(input.gyroHeadingDeg * GYRO_HEADING_GAIN);
    state.quantizedMoveHeadingDeg = quantizeToCardinalDeg(state.gyroHeadingDeg);
    const FacingReference &facing = findClosestFacing(input.compareRollDeg);
    state.snappedCompareRollDeg = facing.rollDeg;
    state.snappedCompareHeadingDeg = facing.headingDeg;
    state.snappedFacingName = facing.name;
    state.peerTruthApplied = false;

    if (input.stationary || input.zupt)
    {
        if (state.hasMovedSinceLastLock)
        {
            state.lockedAfterMove = true;
            state.hasMovedSinceLastLock = false;
        }

        state.speedMps = 0.0f;
        state.velX = 0.0f;
        state.velY = 0.0f;
        state.velZ = 0.0f;
        return;
    }

    if (state.accelMss < DEAD_RECKON_ACC_THRESH_MS2)
    {
        state.speedMps *= DEAD_RECKON_SPEED_DAMP;
        if (state.speedMps < DEAD_RECKON_MIN_SPEED_MPS)
            state.speedMps = 0.0f;
    }
    else
    {
        state.speedMps += state.accelMss * dt;
        if (state.speedMps > DEAD_RECKON_MAX_SPEED_MPS)
            state.speedMps = DEAD_RECKON_MAX_SPEED_MPS;
    }

    if (state.speedMps > DEAD_RECKON_MIN_SPEED_MPS)
    {
        state.hasMovedSinceLastLock = true;
        state.lockedAfterMove = false;
    }

    const float stepMeters = state.speedMps * dt;
    const float headingRad = degToRad(state.quantizedMoveHeadingDeg);
    const float dirX = cosf(headingRad);
    const float dirY = sinf(headingRad);

    state.stepMeters = stepMeters;
    state.velX = state.speedMps * dirX;
    state.velY = state.speedMps * dirY;
    state.velZ = 0.0f;

    state.posX += stepMeters * dirX;
    state.posY += stepMeters * dirY;
    state.posZ = 0.0f;
}

void deadReckoningGetSharedPosition(const DeadReckoningState &state,
                                    bool sharedFrameLocked,
                                    float sharedYawOffsetDeg,
                                    float &x,
                                    float &y,
                                    float &z)
{
    x = state.posX;
    y = state.posY;
    z = state.posZ;

    if (sharedFrameLocked)
        rotate2D(state.posX, state.posY, sharedYawOffsetDeg, x, y);
}

void deadReckoningComputePeerView(const DeadReckoningState &state,
                                  bool sharedFrameLocked,
                                  float sharedYawOffsetDeg,
                                  float localCompareHeadingDeg,
                                  float peerX,
                                  float peerY,
                                  float peerZ,
                                  DeadReckoningPeerView &view)
{
    float myX = state.posX;
    float myY = state.posY;
    float myZ = state.posZ;

    if (sharedFrameLocked)
        rotate2D(state.posX, state.posY, sharedYawOffsetDeg, myX, myY);

    view.dx = peerX - myX;
    view.dy = peerY - myY;
    view.dz = peerZ - myZ;
    view.distance = vecNorm3(view.dx, view.dy, view.dz);
    view.isClose = (view.distance <= PEER_CLOSE_DISTANCE_M);
    view.bearingWorldDeg = atan2f(view.dy, view.dx) * 180.0f / PI;
    view.bearingLocalDeg = wrapAngleDeg(view.bearingWorldDeg - localCompareHeadingDeg);

    if (view.isClose)
    {
        view.directionCode = LINK_PEER_CLOSE;
        view.directionConfidence = 3;
        return;
    }

    const float snappedBearing = quantizeToCardinalDeg(view.bearingLocalDeg);
    view.directionCode = directionCodeFromBearing(snappedBearing);
    view.directionConfidence = confidenceFromBearing(view.bearingLocalDeg);
}

void deadReckoningApplyPeerTruth(DeadReckoningState &state,
                                 bool sharedFrameLocked,
                                 float sharedYawOffsetDeg,
                                 float peerX,
                                 float peerY,
                                 float peerZ,
                                 float peerFacingHeadingDeg,
                                 uint8_t peerDirectionCode,
                                 uint8_t peerConfidence,
                                 bool peerStationary,
                                 bool peerLockedAfterMove,
                                 bool localStationary,
                                 bool localLockedAfterMove,
                                 const DeadReckoningPeerView &localView)
{
    state.peerTruthApplied = false;

    if (peerDirectionCode == LINK_PEER_UNKNOWN || peerDirectionCode == LINK_PEER_CLOSE)
        return;

    const bool peerWinsByConfidence = (peerConfidence > localView.directionConfidence);
    const bool peerWinsByMotionTiebreak =
        (peerConfidence == localView.directionConfidence) &&
        !peerStationary &&
        localStationary;
    const bool peerWinsByLockSignal =
        peerLockedAfterMove &&
        !localLockedAfterMove &&
        (peerConfidence >= localView.directionConfidence);

    if (!(peerWinsByConfidence || peerWinsByMotionTiebreak || peerWinsByLockSignal))
        return;
    if (localView.distance <= 0.001f)
        return;

    const float bearingPeerToMeDeg =
        wrapAngleDeg(peerFacingHeadingDeg + directionOffsetDeg(peerDirectionCode));
    const float bearingMeToPeerDeg = wrapAngleDeg(bearingPeerToMeDeg + 180.0f);
    const float bearingRad = degToRad(bearingMeToPeerDeg);
    const float targetDx = cosf(bearingRad) * localView.distance;
    const float targetDy = sinf(bearingRad) * localView.distance;

    const float targetSharedX = peerX - targetDx;
    const float targetSharedY = peerY - targetDy;

    if (sharedFrameLocked)
        rotate2D(targetSharedX, targetSharedY, -sharedYawOffsetDeg, state.posX, state.posY);
    else
    {
        state.posX = targetSharedX;
        state.posY = targetSharedY;
    }

    state.posZ = 0.0f;
    state.velX = 0.0f;
    state.velY = 0.0f;
    state.velZ = 0.0f;
    state.speedMps *= 0.5f;
    state.peerTruthApplied = true;
}

const char *deadReckoningDirectionName(uint8_t directionCode)
{
    switch (directionCode)
    {
        case LINK_PEER_FORWARD:
            return "forward";
        case LINK_PEER_RIGHT:
            return "right";
        case LINK_PEER_BACK:
            return "back";
        case LINK_PEER_LEFT:
            return "left";
        case LINK_PEER_CLOSE:
            return "close";
        default:
            return "unknown";
    }
}
