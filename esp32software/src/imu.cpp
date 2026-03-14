#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <MadgwickAHRS.h>

#include "imu_math.h"
#include "imu_accel.h"
#include "imu_mag.h"

static Adafruit_ICM20948 icm;
static Madgwick filter;

static bool imu_ok = false;
static bool g_stationary_hold = false;
static unsigned long last_update_us = 0;
static unsigned long last_debug_ms = 0;

static constexpr float G_MSS = 9.80665f;
static constexpr float RAD_TO_DEG_F = 57.2957795f;

// Raw readings
static float raw_ax_mss = 0.0f;
static float raw_ay_mss = 0.0f;
static float raw_az_mss = G_MSS;

static float raw_gx_dps = 0.0f;
static float raw_gy_dps = 0.0f;
static float raw_gz_dps = 0.0f;

static float raw_mx_uT = 0.0f;
static float raw_my_uT = 0.0f;
static float raw_mz_uT = 0.0f;

// Orientation
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float roll_deg = 0.0f;
static float pitch_deg = 0.0f;
static float yaw_deg = 0.0f;

static ImuAccelCal gAccelCal;
static ImuMotionState gMotion;
static ImuMagCal gMagCal;

static bool readRawIMU()
{
    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t mag_event;
    sensors_event_t temp_event;

    icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

    raw_ax_mss = accel_event.acceleration.x - gAccelCal.accelBiasMss[0];
    raw_ay_mss = accel_event.acceleration.y - gAccelCal.accelBiasMss[1];
    raw_az_mss = accel_event.acceleration.z - gAccelCal.accelBiasMss[2];

    raw_gx_dps = gyro_event.gyro.x * RAD_TO_DEG_F - gAccelCal.gyroBiasDps[0];
    raw_gy_dps = gyro_event.gyro.y * RAD_TO_DEG_F - gAccelCal.gyroBiasDps[1];
    raw_gz_dps = gyro_event.gyro.z * RAD_TO_DEG_F - gAccelCal.gyroBiasDps[2];

    imuMagApply(gMagCal,
                mag_event.magnetic.x,
                mag_event.magnetic.y,
                mag_event.magnetic.z,
                raw_mx_uT, raw_my_uT, raw_mz_uT);

    return true;
}

static void calibrateAccelGyro10s()
{
    const unsigned long start = millis();
    float sumGx = 0, sumGy = 0, sumGz = 0;
    float sumAx = 0, sumAy = 0, sumAz = 0;
    int samples = 0;

    Serial.println("Accel/Gyro calibration 10s: keep still");

    while (millis() - start < 10000)
    {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        sensors_event_t temp_event;
        icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

        sumAx += accel_event.acceleration.x;
        sumAy += accel_event.acceleration.y;
        sumAz += accel_event.acceleration.z;

        sumGx += gyro_event.gyro.x * RAD_TO_DEG_F;
        sumGy += gyro_event.gyro.y * RAD_TO_DEG_F;
        sumGz += gyro_event.gyro.z * RAD_TO_DEG_F;
        samples++;

        delay(20);
    }

    imuAccelCalibrateStill(gAccelCal, sumGx, sumGy, sumGz, sumAx, sumAy, sumAz, samples);
}

static void calibrateMag10s()
{
    const unsigned long start = millis();
    imuMagInit(gMagCal);

    Serial.println("Mag calibration 10s: rotate slowly through many angles");

    while (millis() - start < 10000)
    {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        sensors_event_t temp_event;
        icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

        imuMagObserve(gMagCal,
                      mag_event.magnetic.x,
                      mag_event.magnetic.y,
                      mag_event.magnetic.z);

        delay(20);
    }

    imuMagFinish(gMagCal);
}

bool imu_begin()
{
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(20);

    if (!icm.begin_I2C(0x68, &Wire))
    {
        Serial.println("ICM-20948 init failed via Adafruit begin_I2C(0x68)");
        imu_ok = false;
        return false;
    }

    delay(100);

    icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

    imuAccelInit(gAccelCal, gMotion);
    imuMagInit(gMagCal);

    calibrateAccelGyro10s();
    calibrateMag10s();

    filter.begin(100.0f);
    last_update_us = micros();
    imu_ok = true;

    Serial.println("ICM-20948 ready");
    return true;
}

void imu_update()
{
    if (!imu_ok) return;
    if (!readRawIMU()) return;

    const unsigned long now_us = micros();
    float dt = (now_us - last_update_us) * 1e-6f;
    last_update_us = now_us;

    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.01f;

    const float ax_g = raw_ax_mss / G_MSS;
    const float ay_g = raw_ay_mss / G_MSS;
    const float az_g = raw_az_mss / G_MSS;

    filter.update(raw_gx_dps, raw_gy_dps, raw_gz_dps,
                  ax_g, ay_g, az_g,
                  raw_mx_uT, raw_my_uT, raw_mz_uT);

    quat[0] = filter.q0;
    quat[1] = filter.q1;
    quat[2] = filter.q2;
    quat[3] = filter.q3;

    imuMathQuatToEulerDeg(quat, roll_deg, pitch_deg, yaw_deg);

    imuAccelProcess(gMotion,
                    quat,
                    raw_ax_mss, raw_ay_mss, raw_az_mss,
                    raw_gx_dps, raw_gy_dps, raw_gz_dps,
                    dt,
                    g_stationary_hold);
}

void imu_getRawAccel(float &ax, float &ay, float &az)
{
    ax = raw_ax_mss;
    ay = raw_ay_mss;
    az = raw_az_mss;
}

void imu_getRawGyro(float &gx, float &gy, float &gz)
{
    gx = raw_gx_dps;
    gy = raw_gy_dps;
    gz = raw_gz_dps;
}

void imu_getRawMag(float &mx, float &my, float &mz)
{
    mx = raw_mx_uT;
    my = raw_my_uT;
    mz = raw_mz_uT;
}

void imu_getLinearAccel(float &ax, float &ay, float &az)
{
    ax = gMotion.linBody[0];
    ay = gMotion.linBody[1];
    az = gMotion.linBody[2];
}

void imu_getLinearAccelWorld(float &ax, float &ay, float &az)
{
    ax = gMotion.linWorld[0];
    ay = gMotion.linWorld[1];
    az = gMotion.linWorld[2];
}

void imu_getQuat(float &q0, float &q1, float &q2, float &q3)
{
    q0 = quat[0];
    q1 = quat[1];
    q2 = quat[2];
    q3 = quat[3];
}

void imu_getEuler(float &roll, float &pitch, float &yaw)
{
    roll = roll_deg;
    pitch = pitch_deg;
    yaw = yaw_deg;
}

void imu_getPosition(float &x, float &y, float &z)
{
    x = gMotion.posWorld[0];
    y = gMotion.posWorld[1];
    z = gMotion.posWorld[2];
}

void imu_getVelocity(float &x, float &y, float &z)
{
    x = gMotion.velWorld[0];
    y = gMotion.velWorld[1];
    z = gMotion.velWorld[2];
}

void imu_setStationary(bool still)
{
    g_stationary_hold = still;
    if (still)
        imuAccelZeroVelocity(gMotion);
        imu_zeroGyroRate();
}

void imu_zeroVelocity()
{
    imuAccelZeroVelocity(gMotion);
}

void imu_zeroGyroRate()
{
    // Reset pitch (Y) and yaw (Z) bias only
    gAccelCal.gyroBiasDps[1] += raw_gy_dps;
    gAccelCal.gyroBiasDps[2] += raw_gz_dps;

    raw_gy_dps = 0.0f;
    raw_gz_dps = 0.0f;

    // Roll (X axis) is intentionally left unchanged
}

void imu_resetPosition()
{
    imuAccelResetPosition(gMotion);
}

void imu_printDebug()
{
    const unsigned long now = millis();
    if (now - last_debug_ms < 100)
        return;
    last_debug_ms = now;

    float gravity_body[3];
    imuMathComputeGravityBodyFromQuat(quat, gravity_body);

    Serial.println("=== IMU ===");

    Serial.print("RAW ACC m/s^2: ");
    Serial.print(raw_ax_mss, 4); Serial.print(", ");
    Serial.print(raw_ay_mss, 4); Serial.print(", ");
    Serial.println(raw_az_mss, 4);

    Serial.print("RAW GYRO deg/s: ");
    Serial.print(raw_gx_dps, 4); Serial.print(", ");
    Serial.print(raw_gy_dps, 4); Serial.print(", ");
    Serial.println(raw_gz_dps, 4);

    Serial.print("RAW MAG uT: ");
    Serial.print(raw_mx_uT, 4); Serial.print(", ");
    Serial.print(raw_my_uT, 4); Serial.print(", ");
    Serial.println(raw_mz_uT, 4);

    Serial.print("GRAV BODY m/s^2: ");
    Serial.print(gravity_body[0], 4); Serial.print(", ");
    Serial.print(gravity_body[1], 4); Serial.print(", ");
    Serial.println(gravity_body[2], 4);

    Serial.print("LIN ACC BODY m/s^2: ");
    Serial.print(gMotion.linBody[0], 4); Serial.print(", ");
    Serial.print(gMotion.linBody[1], 4); Serial.print(", ");
    Serial.println(gMotion.linBody[2], 4);

    Serial.print("LIN ACC WORLD m/s^2: ");
    Serial.print(gMotion.linWorld[0], 4); Serial.print(", ");
    Serial.print(gMotion.linWorld[1], 4); Serial.print(", ");
    Serial.println(gMotion.linWorld[2], 4);

    Serial.print("RPY deg: ");
    Serial.print(roll_deg, 2); Serial.print(", ");
    Serial.print(pitch_deg, 2); Serial.print(", ");
    Serial.println(yaw_deg, 2);

    Serial.print("POS m: ");
    Serial.print(gMotion.posWorld[0], 4); Serial.print(", ");
    Serial.print(gMotion.posWorld[1], 4); Serial.print(", ");
    Serial.println(gMotion.posWorld[2], 4);

    Serial.print("VEL m/s: ");
    Serial.print(gMotion.velWorld[0], 4); Serial.print(", ");
    Serial.print(gMotion.velWorld[1], 4); Serial.print(", ");
    Serial.println(gMotion.velWorld[2], 4);
}