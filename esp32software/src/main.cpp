#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#include "imu.h"
#include "link.h"
#include "time_sync.h"
#include "imu_accel.h"
#include "debug_config.h"

#define CE_PIN 4
#define CSN_PIN 5

// CHANGE THESE PER DEVICE
#define SELF_ID 2
#define IS_TIME_MASTER 0

#if IS_TIME_MASTER
#define IS_POLLER 1
#else
#define IS_POLLER 0
#endif

RF24 radio(CE_PIN, CSN_PIN);

static uint16_t gSeq = 0;
static StatePacket localPkt = {};
static StatePacket peerPkt = {};
static TimeSyncState gTimeSync;

// ---------- Tunable thresholds ----------
static const float G_MS2 = 9.80665f;
static const float STATIONARY_GYRO_DPS = 2.0f;
static const float STATIONARY_ACC_ERR_MS2 = 0.35f;
static const float ZUPT_GYRO_DPS = 1.2f;
static const float ZUPT_ACC_ERR_MS2 = 0.20f;

static const int STILL_SAMPLES_TO_LATCH = 8;
static const int MOVE_SAMPLES_TO_RELEASE = 4;

// Hard reset after being still a bit
static const unsigned long STILL_RESET_MS = 1200;

// Shared-frame alignment
static bool gInitialYawLocked = false;
static float gInitialYawDeg = 0.0f;
static bool gSharedFrameLocked = false;
static float gSharedYawOffsetDeg = 0.0f;

// ---------- Local estimator/debug state ----------
struct MotionState
{
    // Raw sensor/debug
    float rawAx = 0, rawAy = 0, rawAz = 0;
    float rawGx = 0, rawGy = 0, rawGz = 0;
    float rawMx = 0, rawMy = 0, rawMz = 0;

    float linAxB = 0, linAyB = 0, linAzB = 0;
    float linAxW = 0, linAyW = 0, linAzW = 0;

    // Raw pose from IMU layer, shifted so startup = origin
    float rawPosX = 0, rawPosY = 0, rawPosZ = 0;
    float rawVelX = 0, rawVelY = 0, rawVelZ = 0;

    // Clamped pose
    float clampPosX = 0, clampPosY = 0, clampPosZ = 0;
    float clampVelX = 0, clampVelY = 0, clampVelZ = 0;

    // Persistent correction offset:
    // corrected = raw + correction
    float corrX = 0, corrY = 0, corrZ = 0;

    // Startup origin from IMU layer
    float originX = 0, originY = 0, originZ = 0;
    bool originSet = false;

    bool stationary = false;
    bool zupt = false;

    float gyroNorm = 0;
    float accNorm = 0;
    float accErr = 0;

    int stillCount = 0;
    int moveCount = 0;

    bool imuHoldActive = false;
    unsigned long stillSinceMs = 0;
    bool resetDoneThisStillness = false;
    bool prevHold;

    float rawRoll = 0, rawPitch = 0, rawYaw = 0;
    float clampRoll = 0, clampPitch = 0, clampYaw = 0;
    float corrRoll = 0, corrPitch = 0, corrYaw = 0;

    float orientOffRoll = 0, orientOffPitch = 0, orientOffYaw = 0;

    float holdRollRef = 0, holdPitchRef = 0, holdYawRef = 0;
    bool angleHoldRefValid;

    bool holdLatched = false;
};

static MotionState gMotion;

// ---------- Peer relative/debug state ----------
struct RelativeState
{
    float dx = 0, dy = 0, dz = 0;
    float distance = 0;
    float bearingWorldDeg = 0;
    float bearingLocalDeg = 0;
};

static RelativeState gRel;

// ---------- Timing ----------
static unsigned long gLastDebugMs = 0;

// ---------- Utility ----------
static float wrapAngleDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

static float vecNorm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float degToRad(float deg)
{
    return deg * PI / 180.0f;
}

static void rotate2D(float x, float y, float yawDeg, float &outX, float &outY)
{
    const float r = degToRad(yawDeg);
    const float c = cosf(r);
    const float s = sinf(r);
    outX = c * x - s * y;
    outY = s * x + c * y;
}

static uint8_t buildFlags()
{
    uint8_t flags = 0;

    // bit0 = stance / stationary-ish
    // bit1 = ZUPT active
    // bit2 = stationary
    // bit3 = heading / time reliable
    // bit4 = reserved
    // bit5 = reserved
    // bit6 = initial yaw valid / shared frame info valid
    // bit7 = shared frame locked locally

    if (gMotion.stationary)
        flags |= (1 << 0);
    if (gMotion.zupt)
        flags |= (1 << 1);
    if (gMotion.stationary)
        flags |= (1 << 2);

#if IS_TIME_MASTER
    flags |= (1 << 3);
#else
    if (timeSyncLocked(gTimeSync))
        flags |= (1 << 3);
#endif

    if (gInitialYawLocked)
        flags |= (1 << 6);
    if (gSharedFrameLocked)
        flags |= (1 << 7);

    return flags;
}

static uint32_t sharedNowUs()
{
    const uint32_t nowLocal = micros();

#if IS_TIME_MASTER
    return nowLocal;
#else
    return timeSyncNowUs(gTimeSync, nowLocal);
#endif
}

static void captureImuState()
{
    imu_getRawAccel(gMotion.rawAx, gMotion.rawAy, gMotion.rawAz);
    imu_getRawGyro(gMotion.rawGx, gMotion.rawGy, gMotion.rawGz);
    imu_getRawMag(gMotion.rawMx, gMotion.rawMy, gMotion.rawMz);

    imu_getLinearAccel(gMotion.linAxB, gMotion.linAyB, gMotion.linAzB);
    imu_getLinearAccelWorld(gMotion.linAxW, gMotion.linAyW, gMotion.linAzW);
    imu_getEuler(gMotion.rawRoll, gMotion.rawPitch, gMotion.rawYaw);

    float px, py, pz;
    float vx, vy, vz;

    imu_getPosition(px, py, pz);
    imu_getVelocity(vx, vy, vz);

    if (!gMotion.originSet)
    {
        gMotion.originSet = true;
        gMotion.originX = px;
        gMotion.originY = py;
        gMotion.originZ = pz;
    }

    gMotion.rawPosX = px - gMotion.originX;
    gMotion.rawPosY = py - gMotion.originY;
    gMotion.rawPosZ = pz - gMotion.originZ;

    gMotion.rawVelX = vx;
    gMotion.rawVelY = vy;
    gMotion.rawVelZ = vz;
}

static void detectMotionFlags()
{
    gMotion.gyroNorm = vecNorm3(gMotion.rawGx, gMotion.rawGy, gMotion.rawGz);
    gMotion.accNorm = vecNorm3(gMotion.rawAx, gMotion.rawAy, gMotion.rawAz);
    gMotion.accErr = fabsf(gMotion.accNorm - G_MS2);

    const bool stillCandidate =
        (gMotion.gyroNorm < STATIONARY_GYRO_DPS) &&
        (gMotion.accErr < STATIONARY_ACC_ERR_MS2);

    if (stillCandidate)
    {
        gMotion.stillCount++;
        gMotion.moveCount = 0;
    }
    else
    {
        gMotion.moveCount++;
        gMotion.stillCount = 0;
    }

    if (gMotion.stillCount >= STILL_SAMPLES_TO_LATCH)
        gMotion.stationary = true;
    else if (gMotion.moveCount >= MOVE_SAMPLES_TO_RELEASE)
        gMotion.stationary = false;

    gMotion.zupt =
        (gMotion.gyroNorm < ZUPT_GYRO_DPS) &&
        (gMotion.accErr < ZUPT_ACC_ERR_MS2);
}

static void maybeLockInitialYaw()
{
    if (gInitialYawLocked)
        return;

    if (!(gMotion.stationary || gMotion.zupt))
        return;

    if (gMotion.stillSinceMs == 0)
        return;

    if (millis() - gMotion.stillSinceMs < STILL_RESET_MS)
        return;

    gInitialYawDeg = gMotion.rawYaw;
    gInitialYawLocked = true;

#if IS_TIME_MASTER
    gSharedYawOffsetDeg = 0.0f;
    gSharedFrameLocked = true;
#endif
}

static void maybeLockSharedFrameFromPeer()
{
#if !IS_TIME_MASTER
    if (gSharedFrameLocked)
        return;

    if (!gInitialYawLocked)
        return;

    if ((peerPkt.flags & (1 << 6)) == 0)
        return;

    gSharedYawOffsetDeg = wrapAngleDeg(peerPkt.initYawDeg - gInitialYawDeg);
    gSharedFrameLocked = true;
#endif
}
static void updateStationaryHold()
{
    const uint32_t now = millis();

    // Raw request from detectors
    const bool holdRequest = (gMotion.stationary || gMotion.zupt);
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold requested: %d\n", holdRequest);
#endif
    // Movement is the absence of a hold request from the stationary/ZUPT detectors.
    const bool moved = !holdRequest;
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Moved: %d\n", moved);
#endif

    if (holdRequest)
        gMotion.holdLatched = true;
    else if (moved)
        gMotion.holdLatched = false;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold latched: %d", gMotion.holdLatched);
#endif

    const bool hold = gMotion.holdLatched;
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold: %d\n", hold);
#endif

    imu_setStationary(hold);
    gMotion.imuHoldActive = hold;

    const bool enteredHold = (hold && !gMotion.prevHold);
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Entered hold: %d\n", enteredHold);
#endif

    const bool exitedHold = (!hold && gMotion.prevHold);

    // Current corrected orientation from raw + correction
    const float correctedRoll = wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll);
    const float correctedPitch = wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch);
    const float correctedYaw = wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw);

    // Debug output
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf(
        "PITCH DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawPitch,
        gMotion.corrPitch,
        correctedPitch);

    Serial.printf(
        "ROLL DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawRoll,
        gMotion.corrRoll,
        correctedRoll);

    Serial.printf(
        "YAW DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawYaw,
        gMotion.corrYaw,
        correctedYaw);

    Serial.printf(
        "PRE-APPLY | holdRef=(%.2f, %.2f, %.2f) clamp=(%.2f, %.2f, %.2f)\n",
        gMotion.holdRollRef,
        gMotion.holdPitchRef,
        gMotion.holdYawRef,
        gMotion.clampRoll,
        gMotion.clampPitch,
        gMotion.clampYaw);
#endif

    if (enteredHold)
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
        Serial.println("========= RECALCULATING HOLD =========");

        Serial.printf(
            "NEW HOLD REF | roll: %.2f  pitch: %.2f  yaw: %.2f\n",
            correctedRoll,
            correctedPitch,
            correctedYaw);
#endif

        gMotion.holdRollRef = correctedRoll;
        gMotion.holdPitchRef = correctedPitch;
        gMotion.holdYawRef = correctedYaw;

        gMotion.angleHoldRefValid = true;
        gMotion.stillSinceMs = now;
        gMotion.resetDoneThisStillness = false;

        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        // Freeze current corrected orientation immediately
        gMotion.clampRoll = correctedRoll;
        gMotion.clampPitch = correctedPitch;
        gMotion.clampYaw = correctedYaw;
    }

    if (hold)
    {
        // Keep clamp velocity zero while held
        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        if (!gMotion.resetDoneThisStillness &&
            (now - gMotion.stillSinceMs >= STILL_RESET_MS))
        {
            // Lock clamp position where it currently is
            gMotion.clampPosX = gMotion.rawPosX + gMotion.corrX;
            gMotion.clampPosY = gMotion.rawPosY + gMotion.corrY;
            gMotion.clampPosZ = gMotion.rawPosZ + gMotion.corrZ;

            // Recompute correction so corrected raw position matches held clamp
            gMotion.corrX = gMotion.clampPosX - gMotion.rawPosX;
            gMotion.corrY = gMotion.clampPosY - gMotion.rawPosY;
            gMotion.corrZ = gMotion.clampPosZ - gMotion.rawPosZ;

            gMotion.clampVelX = 0.0f;
            gMotion.clampVelY = 0.0f;
            gMotion.clampVelZ = 0.0f;

            // Preserve the orientation captured when hold was first entered.
            gMotion.clampRoll = gMotion.holdRollRef;
            gMotion.clampPitch = gMotion.holdPitchRef;
            gMotion.clampYaw = gMotion.holdYawRef;

            gMotion.corrRoll = wrapAngleDeg(gMotion.clampRoll - gMotion.rawRoll);
            gMotion.corrPitch = wrapAngleDeg(gMotion.clampPitch - gMotion.rawPitch);
            gMotion.corrYaw = wrapAngleDeg(gMotion.clampYaw - gMotion.rawYaw);

            gMotion.resetDoneThisStillness = true;
        }
    }
    else
    {
        if (exitedHold)
        {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.println("========= HOLD RELEASED: MOVEMENT DETECTED =========");
#endif

            gMotion.stillSinceMs = 0;
            gMotion.resetDoneThisStillness = false;
        }
    }

    gMotion.prevHold = hold;
}
static void applyClamps()
{
    const float correctedX = gMotion.rawPosX + gMotion.corrX;
    const float correctedY = gMotion.rawPosY + gMotion.corrY;
    const float correctedZ = gMotion.rawPosZ + gMotion.corrZ;

    const float correctedRoll = wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll);
    const float correctedPitch = wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch);
    const float correctedYaw = wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw);

    const float gyroMagDps = sqrtf(
        gMotion.rawGx * gMotion.rawGx +
        gMotion.rawGy * gMotion.rawGy +
        gMotion.rawGz * gMotion.rawGz);

    const bool holdAnglesToStoredRef =
        (gMotion.stationary || gMotion.zupt) &&
        gMotion.angleHoldRefValid &&
        (gyroMagDps < GYRO_NOISE_THRESH_DPS);

    if (gMotion.stationary || gMotion.zupt)
    {
        // Hold position fixed at last clamped point
        gMotion.corrX = gMotion.clampPosX - gMotion.rawPosX;
        gMotion.corrY = gMotion.clampPosY - gMotion.rawPosY;
        gMotion.corrZ = gMotion.clampPosZ - gMotion.rawPosZ;

        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        // Only hold angles if angular motion is just noise
        if (holdAnglesToStoredRef)
        {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.println("ANGLE HOLD ACTIVE");
            Serial.printf(
                "APPLY INPUT | gyroMag=%.3f holdValid=%d stationary=%d zupt=%d\n",
                gyroMagDps,
                gMotion.angleHoldRefValid,
                gMotion.stationary,
                gMotion.zupt);

            Serial.printf(
                "ROLL HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawRoll,
                gMotion.holdRollRef,
                wrapAngleDeg(gMotion.holdRollRef - gMotion.rawRoll));

            Serial.printf(
                "PITCH HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawPitch,
                gMotion.holdPitchRef,
                wrapAngleDeg(gMotion.holdPitchRef - gMotion.rawPitch));

            Serial.printf(
                "YAW HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawYaw,
                gMotion.holdYawRef,
                wrapAngleDeg(gMotion.holdYawRef - gMotion.rawYaw));
#endif

            gMotion.corrRoll = wrapAngleDeg(gMotion.holdRollRef - gMotion.rawRoll);
            gMotion.corrPitch = wrapAngleDeg(gMotion.holdPitchRef - gMotion.rawPitch);
            gMotion.corrYaw = wrapAngleDeg(gMotion.holdYawRef - gMotion.rawYaw);

            gMotion.clampRoll = gMotion.holdRollRef;
            gMotion.clampPitch = gMotion.holdPitchRef;
            gMotion.clampYaw = gMotion.holdYawRef;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.printf(
                "APPLY OUTPUT | corr=(%.2f, %.2f, %.2f) corrected=(%.2f, %.2f, %.2f) clamp=(%.2f, %.2f, %.2f)\n",
                gMotion.corrRoll,
                gMotion.corrPitch,
                gMotion.corrYaw,
                wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll),
                wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch),
                wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw),
                gMotion.clampRoll,
                gMotion.clampPitch,
                gMotion.clampYaw);
#endif
        }
        else
        {
            // Let orientation move if gyro says this is real motion
            gMotion.corrRoll = 0.0f;
            gMotion.corrPitch = 0.0f;
            gMotion.corrYaw = 0.0f;
            gMotion.clampRoll = correctedRoll;
            gMotion.clampPitch = correctedPitch;
            gMotion.clampYaw = correctedYaw;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.printf(
                "ANGLE HOLD BYPASSED | gyroMag=%.3f holdValid=%d corrected=(%.2f, %.2f, %.2f)\n",
                gyroMagDps,
                gMotion.angleHoldRefValid,
                correctedRoll,
                correctedPitch,
                correctedYaw);
#endif
        }

        // imu_zeroVelocity();
    }
    else
    {
        // During motion, let corrected position evolve
        gMotion.clampPosX = correctedX;
        gMotion.clampPosY = correctedY;
        gMotion.clampPosZ = correctedZ;

        gMotion.clampVelX = gMotion.rawVelX;
        gMotion.clampVelY = gMotion.rawVelY;
        gMotion.clampVelZ = gMotion.rawVelZ;

        // During motion, let corrected orientation evolve
        gMotion.corrRoll = 0.0f;
        gMotion.corrPitch = 0.0f;
        gMotion.corrYaw = 0.0f;
        gMotion.clampRoll = correctedRoll;
        gMotion.clampPitch = correctedPitch;
        gMotion.clampYaw = correctedYaw;
    }
}

static void fillLocalPacket(StatePacket &pkt)
{
    pkt.deviceId = SELF_ID;
    pkt.flags = buildFlags();
    pkt.seq = gSeq++;
    pkt.timeUs = sharedNowUs();

    pkt.yawDeg = gMotion.clampYaw; // TODO CLAMP YAW
    pkt.initYawDeg = gInitialYawDeg;

    float tx = gMotion.clampPosX;
    float ty = gMotion.clampPosY;
    float tz = gMotion.clampPosZ;

    if (gSharedFrameLocked)
        rotate2D(gMotion.clampPosX, gMotion.clampPosY, gSharedYawOffsetDeg, tx, ty);

    pkt.posX = tx;
    pkt.posY = ty;
    pkt.posZ = tz;

    pkt.speedMps = vecNorm3(gMotion.clampVelX, gMotion.clampVelY, gMotion.clampVelZ);
}

static void computeRelativeDirection()
{
    float myX = gMotion.clampPosX;
    float myY = gMotion.clampPosY;

    if (gSharedFrameLocked)
        rotate2D(gMotion.clampPosX, gMotion.clampPosY, gSharedYawOffsetDeg, myX, myY);

    gRel.dx = peerPkt.posX - myX;
    gRel.dy = peerPkt.posY - myY;
    gRel.dz = peerPkt.posZ - gMotion.clampPosZ;

    gRel.distance = vecNorm3(gRel.dx, gRel.dy, gRel.dz);

    gRel.bearingWorldDeg = atan2f(gRel.dy, gRel.dx) * 180.0f / PI;
    gRel.bearingLocalDeg = wrapAngleDeg(gRel.bearingWorldDeg - gMotion.clampYaw); // TODO CLAMP YAW
}

static void printDebug()
{
#if !(DEBUG_SERIAL_ENABLE && DEBUG_MAIN_STATE)
    return;
#else
    Serial.println();
    Serial.println("========== DEBUG ==========");
    Serial.print("SELF_ID=");
    Serial.print(SELF_ID);
    Serial.print(" role=");
#if IS_TIME_MASTER
    Serial.println("TIME_MASTER / POLLER");
#else
    Serial.println("TIME_SLAVE / RESPONDER");
#endif

    Serial.println("-- Time Sync --");
#if IS_TIME_MASTER
    Serial.print("shared_now_us=");
    Serial.println(sharedNowUs());
    Serial.println("locked=yes (master clock)");
#else
    Serial.print("shared_now_us=");
    Serial.println(sharedNowUs());
    Serial.print("offset_us=");
    Serial.println(timeSyncOffsetUs(gTimeSync, micros()));
    Serial.print("rate_ppm=");
    Serial.println(timeSyncRatePpm(gTimeSync), 3);
    Serial.print("locked=");
    Serial.println(timeSyncLocked(gTimeSync) ? "yes" : "no");
#endif

    Serial.println("-- Raw Sensors --");
    Serial.print("accel_raw=(");
    Serial.print(gMotion.rawAx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawAy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawAz, 3);
    Serial.println(")");

    Serial.print("gyro_raw=(");
    Serial.print(gMotion.rawGx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawGy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawGz, 3);
    Serial.println(")");

    Serial.print("mag_raw=(");
    Serial.print(gMotion.rawMx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawMy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawMz, 3);
    Serial.println(")");

    Serial.println("-- Fused IMU --");
    Serial.print("linacc_body=(");
    Serial.print(gMotion.linAxB, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAyB, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAzB, 3);
    Serial.println(")");

    Serial.print("linacc_world=(");
    Serial.print(gMotion.linAxW, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAyW, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAzW, 3);
    Serial.println(")");

    Serial.print("euler_deg=(roll=");
    Serial.print(gMotion.clampRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.clampPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.clampYaw, 2);
    Serial.println(")");

    Serial.println("-- Motion Flags --");
    Serial.print("gyro_norm_dps=");
    Serial.println(gMotion.gyroNorm, 3);
    Serial.print("acc_norm_ms2=");
    Serial.println(gMotion.accNorm, 3);
    Serial.print("acc_err_ms2=");
    Serial.println(gMotion.accErr, 3);

    Serial.print("stationary=");
    Serial.println(gMotion.stationary ? "yes" : "no");
    Serial.print("zupt=");
    Serial.println(gMotion.zupt ? "yes" : "no");
    Serial.print("imu_hold=");
    Serial.println(gMotion.imuHoldActive ? "yes" : "no");
    Serial.print("still_count=");
    Serial.println(gMotion.stillCount);
    Serial.print("move_count=");
    Serial.println(gMotion.moveCount);

    Serial.println("-- Shared Frame --");
    Serial.print("initial_yaw_locked=");
    Serial.println(gInitialYawLocked ? "yes" : "no");
    Serial.print("initial_yaw_deg=");
    Serial.println(gInitialYawDeg, 2);
    Serial.print("shared_frame_locked=");
    Serial.println(gSharedFrameLocked ? "yes" : "no");
    Serial.print("shared_yaw_offset_deg=");
    Serial.println(gSharedYawOffsetDeg, 2);

    Serial.println("-- Position / Velocity --");
    Serial.print("raw_pos=(");
    Serial.print(gMotion.rawPosX, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawPosY, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawPosZ, 3);
    Serial.println(")");

    Serial.print("raw_vel=(");
    Serial.print(gMotion.rawVelX, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawVelY, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawVelZ, 3);
    Serial.println(")");

    Serial.print("corr_offset=(");
    Serial.print(gMotion.corrX, 3);
    Serial.print(", ");
    Serial.print(gMotion.corrY, 3);
    Serial.print(", ");
    Serial.print(gMotion.corrZ, 3);
    Serial.println(")");

    Serial.print("clamped_pos=(");
    Serial.print(gMotion.clampPosX, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampPosY, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampPosZ, 3);
    Serial.println(")");

    Serial.print("clamped_vel=(");
    Serial.print(gMotion.clampVelX, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampVelY, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampVelZ, 3);
    Serial.println(")");

    Serial.println("-- Peer State --");
    Serial.print("peer_id=");
    Serial.println(peerPkt.deviceId);
    Serial.print("peer_t_us=");
    Serial.println(peerPkt.timeUs);
    Serial.print("peer_pos=(");
    Serial.print(peerPkt.posX, 3);
    Serial.print(", ");
    Serial.print(peerPkt.posY, 3);
    Serial.print(", ");
    Serial.print(peerPkt.posZ, 3);
    Serial.println(")");
    Serial.print("peer_yaw_deg=");
    Serial.println(peerPkt.yawDeg, 2);
    Serial.print("peer_init_yaw_deg=");
    Serial.println(peerPkt.initYawDeg, 2);
    Serial.print("peer_speed_mps=");
    Serial.println(peerPkt.speedMps, 3);

    Serial.println("-- Relative To Peer --");
    Serial.print("delta_xyz=(");
    Serial.print(gRel.dx, 3);
    Serial.print(", ");
    Serial.print(gRel.dy, 3);
    Serial.print(", ");
    Serial.print(gRel.dz, 3);
    Serial.println(")");
    Serial.print("distance_m=");
    Serial.println(gRel.distance, 3);
    Serial.print("bearing_world_deg=");
    Serial.println(gRel.bearingWorldDeg, 2);
    Serial.print("bearing_local_deg=");
    Serial.println(gRel.bearingLocalDeg, 2);

    Serial.print("euler_raw_deg=(roll=");
    Serial.print(gMotion.rawRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.rawPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.rawYaw, 2);
    Serial.println(")");

    Serial.print("euler_clamped_deg=(roll=");
    Serial.print(gMotion.clampRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.clampPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.clampYaw, 2);
    Serial.println(")");

    Serial.println("===========================");
    Serial.println();
#endif
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPI.begin(18, 19, 23, 5);

    if (!imu_begin())
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
        Serial.println("imu_begin() failed");
#endif
        while (1)
        {
        }
    }

    if (!radio.begin())
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
        Serial.println("radio.begin() failed");
#endif
        while (1)
        {
        }
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);

    timeSyncBegin(gTimeSync, IS_TIME_MASTER != 0);

#if IS_POLLER
    linkBegin(radio, LINK_ROLE_POLLER, SELF_ID);
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
    Serial.print("Started as TIME MASTER / POLLER, SELF_ID=");
    Serial.println(SELF_ID);
#endif
#else
    linkBegin(radio, LINK_ROLE_RESPONDER, SELF_ID);
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
    Serial.print("Started as TIME SLAVE / RESPONDER, SELF_ID=");
    Serial.println(SELF_ID);
#endif
#endif

    gLastDebugMs = millis();
}

void loop()
{
    imu_update();
    captureImuState();
    detectMotionFlags();
    updateStationaryHold();
    maybeLockInitialYaw();
    applyClamps();

    fillLocalPacket(localPkt);
    linkSetLocalState(localPkt);

#if IS_POLLER
    if (linkExchange(radio, peerPkt))
    {
        computeRelativeDirection();
    }
#else
    if (linkPollResponder(radio, peerPkt))
    {
        const uint32_t localRxUs = micros();

        if (peerPkt.deviceId == 1)
            timeSyncObserveMaster(gTimeSync, localRxUs, peerPkt.timeUs);

        maybeLockSharedFrameFromPeer();
        computeRelativeDirection();
    }
#endif

#if IS_POLLER
    // Master always defines the shared frame
    if (gInitialYawLocked && !gSharedFrameLocked)
    {
        gSharedYawOffsetDeg = 0.0f;
        gSharedFrameLocked = true;
    }
#endif

    if (millis() - gLastDebugMs >= 1000)
    {
        gLastDebugMs = millis();
        printDebug();
    }

#if IS_POLLER
    delay(20);
#else
    delay(2);
#endif
}
