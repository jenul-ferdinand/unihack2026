#include "imu_accel.h"
#include "imu_math.h"
#include <Arduino.h>
#include <math.h>

static constexpr float G_MSS = 9.80665f;
static constexpr float ACC_DEADBAND_MSS = 0.25f;

// Very aggressive gating
static constexpr float BURST_ACC_THRESH_MSS = 1.20f;
static constexpr float BURST_GYRO_THRESH_DPS = 12.0f;
static constexpr float EXIT_BURST_ACC_THRESH_MSS = 0.35f;
static constexpr float EXIT_BURST_GYRO_THRESH_DPS = 3.0f;

// Per your request, treat motion under 1 m/s as noise.
static constexpr float MIN_VALID_SPEED_MPS = 0.01f;
static constexpr float MAX_VALID_SPEED_MPS = 100.0f;

void imuAccelInit(ImuAccelCal &cal, ImuMotionState &motion)
{
    cal.gyroBiasDps[0] = cal.gyroBiasDps[1] = cal.gyroBiasDps[2] = 0.0f;
    cal.accelBiasMss[0] = cal.accelBiasMss[1] = cal.accelBiasMss[2] = 0.0f;

    for (int i = 0; i < 3; ++i)
    {
        motion.linBody[i] = 0.0f;
        motion.linWorld[i] = 0.0f;
        motion.velWorld[i] = 0.0f;
        motion.posWorld[i] = 0.0f;
    }

    motion.motionBurst = false;
    motion.travelMeters = 0.0f;
    motion.pendingMeters = 0.0f;
}

void imuAccelCalibrateStill(ImuAccelCal &cal,
                            float sumGx, float sumGy, float sumGz,
                            float sumAx, float sumAy, float sumAz,
                            int samples)
{
    if (samples <= 0)
        return;

    cal.gyroBiasDps[0] = sumGx / samples;
    cal.gyroBiasDps[1] = sumGy / samples;
    cal.gyroBiasDps[2] = sumGz / samples;

    cal.accelBiasMss[0] = sumAx / samples;
    cal.accelBiasMss[1] = sumAy / samples;
    cal.accelBiasMss[2] = (sumAz / samples) - G_MSS;
}

void imuAccelProcess(ImuMotionState &motion,
                     const float quat[4],
                     float rawAx, float rawAy, float rawAz,
                     float rawGx, float rawGy, float rawGz,
                     float dt,
                     bool stationaryHold)
{
    float gravityBody[3];
    imuMathComputeGravityBodyFromQuat(quat, gravityBody);

    motion.linBody[0] = rawAx - gravityBody[0];
    motion.linBody[1] = rawAy - gravityBody[1];
    motion.linBody[2] = rawAz - gravityBody[2];
    imuMathApplyDeadband(motion.linBody, ACC_DEADBAND_MSS);

    imuMathRotateVectorByQuat(quat, motion.linBody, motion.linWorld);
    imuMathApplyDeadband(motion.linWorld, ACC_DEADBAND_MSS);

    const float linAccNorm = imuMathNorm3(
        motion.linBody[0], motion.linBody[1], motion.linBody[2]);
    const float gyroNorm = imuMathNorm3(rawGx, rawGy, rawGz);

#if IMU_DEBUG
    Serial.println();
    Serial.println("====== IMU ACCEL DEBUG ======");
    Serial.print("dt: ");
    Serial.println(dt, 6);

    Serial.print("linBody: (");
    Serial.print(motion.linBody[0], 4);
    Serial.print(", ");
    Serial.print(motion.linBody[1], 4);
    Serial.print(", ");
    Serial.print(motion.linBody[2], 4);
    Serial.println(")");

    Serial.print("linWorld: (");
    Serial.print(motion.linWorld[0], 4);
    Serial.print(", ");
    Serial.print(motion.linWorld[1], 4);
    Serial.print(", ");
    Serial.print(motion.linWorld[2], 4);
    Serial.println(")");

    Serial.print("linAccNorm: ");
    Serial.println(linAccNorm, 4);

    Serial.print("gyroNorm: ");
    Serial.println(gyroNorm, 4);

    Serial.print("velWorld BEFORE: (");
    Serial.print(motion.velWorld[0], 4);
    Serial.print(", ");
    Serial.print(motion.velWorld[1], 4);
    Serial.print(", ");
    Serial.print(motion.velWorld[2], 4);
    Serial.println(")");
#endif

    if (!motion.motionBurst)
    {
        if (linAccNorm > BURST_ACC_THRESH_MSS || gyroNorm > BURST_GYRO_THRESH_DPS)
            motion.motionBurst = true;
    }
    else
    {
        if (linAccNorm < EXIT_BURST_ACC_THRESH_MSS && gyroNorm < EXIT_BURST_GYRO_THRESH_DPS)
            motion.motionBurst = false;
    }

#if IMU_DEBUG
    Serial.print("motionBurst: ");
    Serial.println(motion.motionBurst ? "true" : "false");

    Serial.print("stationaryHold: ");
    Serial.println(stationaryHold ? "true" : "false");
#endif

    if (stationaryHold)
    {
        motion.velWorld[0] = 0.0f;
        motion.velWorld[1] = 0.0f;
        motion.velWorld[2] = 0.0f;

#if IMU_DEBUG
        Serial.println("stationaryHold active -> velocity forced to zero");
        Serial.print("velWorld CLAMPED: (");
        Serial.print(motion.velWorld[0], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[1], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[2], 4);
        Serial.println(")");
#endif
        return;
    }

    if (!motion.motionBurst)
    {
        motion.linWorld[0] = 0.0f;
        motion.linWorld[1] = 0.0f;
        motion.linWorld[2] = 0.0f;

        motion.velWorld[0] *= 0.60f;
        motion.velWorld[1] *= 0.60f;
        motion.velWorld[2] *= 0.60f;

        if (fabsf(motion.velWorld[0]) < 0.02f)
            motion.velWorld[0] = 0.0f;
        if (fabsf(motion.velWorld[1]) < 0.02f)
            motion.velWorld[1] = 0.0f;
        if (fabsf(motion.velWorld[2]) < 0.02f)
            motion.velWorld[2] = 0.0f;

#if IMU_DEBUG
        Serial.println("No motion burst -> damping velocity");
        Serial.print("velWorld DAMPED: (");
        Serial.print(motion.velWorld[0], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[1], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[2], 4);
        Serial.println(")");
#endif
        return;
    }

    for (int i = 0; i < 3; ++i)
        motion.velWorld[i] += motion.linWorld[i] * dt;

    const float speedNorm = imuMathNorm3(
        motion.velWorld[0], motion.velWorld[1], motion.velWorld[2]);

#if IMU_DEBUG
    Serial.print("velWorld AFTER: (");
    Serial.print(motion.velWorld[0], 4);
    Serial.print(", ");
    Serial.print(motion.velWorld[1], 4);
    Serial.print(", ");
    Serial.print(motion.velWorld[2], 4);
    Serial.println(")");

    Serial.print("speedNorm: ");
    Serial.println(speedNorm, 4);
#endif

    if (speedNorm < MIN_VALID_SPEED_MPS || speedNorm > MAX_VALID_SPEED_MPS)
    {
        motion.velWorld[0] = 0.0f;
        motion.velWorld[1] = 0.0f;
        motion.velWorld[2] = 0.0f;

        motion.linWorld[0] = 0.0f;
        motion.linWorld[1] = 0.0f;
        motion.linWorld[2] = 0.0f;

        motion.motionBurst = false;

#if IMU_DEBUG
        Serial.println("speedNorm outside valid range -> reset velocity and accel");
        Serial.print("velWorld RESET: (");
        Serial.print(motion.velWorld[0], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[1], 4);
        Serial.print(", ");
        Serial.print(motion.velWorld[2], 4);
        Serial.println(")");
#endif
        return;
    }

    // Keep position if you still want it for debugging
    motion.posWorld[0] += motion.velWorld[0] * dt;
    motion.posWorld[1] += motion.velWorld[1] * dt;
    motion.posWorld[2] += motion.velWorld[2] * dt;

    // Debug-only clamp calculation
    float posNorm = imuMathNorm3(
        motion.posWorld[0], motion.posWorld[1], motion.posWorld[2]);

    bool withinClampRadius = (posNorm <= CLAMP_RADIUS_M);

    float clampedPos[3] = {
        motion.posWorld[0],
        motion.posWorld[1],
        motion.posWorld[2]};

    if (!withinClampRadius && posNorm > 0.0001f)
    {
        const float scale = CLAMP_RADIUS_M / posNorm;
        clampedPos[0] = motion.posWorld[0] * scale;
        clampedPos[1] = motion.posWorld[1] * scale;
        clampedPos[2] = motion.posWorld[2] * scale;
    }

#if IMU_DEBUG
    Serial.print("posWorld RAW: (");
    Serial.print(motion.posWorld[0], 4);
    Serial.print(", ");
    Serial.print(motion.posWorld[1], 4);
    Serial.print(", ");
    Serial.print(motion.posWorld[2], 4);
    Serial.println(")");

    Serial.print("posNorm: ");
    Serial.println(posNorm, 4);

    Serial.print("within clamp radius (");
    Serial.print(CLAMP_RADIUS_M, 3);
    Serial.print(" m): ");
    Serial.println(withinClampRadius ? "true" : "false");

    Serial.print("clampedPos SET TO: (");
    Serial.print(clampedPos[0], 4);
    Serial.print(", ");
    Serial.print(clampedPos[1], 4);
    Serial.print(", ");
    Serial.print(clampedPos[2], 4);
    Serial.println(")");
#endif

    // This is the magnitude of distance travelled during this frame
    const float frameDistance = speedNorm * dt;

    motion.travelMeters += frameDistance;
    motion.pendingMeters += frameDistance;

#if IMU_DEBUG
    Serial.print("frameDistance: ");
    Serial.println(frameDistance, 4);

    Serial.print("travelMeters: ");
    Serial.println(motion.travelMeters, 4);

    Serial.print("pendingMeters: ");
    Serial.println(motion.pendingMeters, 4);

    Serial.println("============================");
#endif

    motion.posWorld[0] = clampedPos[0];
    motion.posWorld[1] = clampedPos[1];
    motion.posWorld[2] = clampedPos[2];
}

bool imuAccelConsumeMeter(ImuMotionState &motion)
{
    if (motion.pendingMeters >= 1.0f)
    {
        motion.pendingMeters -= 1.0f;
        return true;
    }
    return false;
}

void imuAccelZeroVelocity(ImuMotionState &motion)
{
    motion.velWorld[0] = 0.0f;
    motion.velWorld[1] = 0.0f;
    motion.velWorld[2] = 0.0f;
}

void imuAccelResetPosition(ImuMotionState &motion)
{
    motion.velWorld[0] = 0.0f;
    motion.velWorld[1] = 0.0f;
    motion.velWorld[2] = 0.0f;

    motion.posWorld[0] = 0.0f;
    motion.posWorld[1] = 0.0f;
    motion.posWorld[2] = 0.0f;
}