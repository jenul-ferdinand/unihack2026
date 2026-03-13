#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <MadgwickAHRS.h>

static Adafruit_ICM20948 icm;
static Madgwick filter;

static bool imu_ok = false;
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
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w x y z
static float roll_deg = 0.0f;
static float pitch_deg = 0.0f;
static float yaw_deg = 0.0f;

// Linear accel
static float accel_lin_body[3] = {0.0f, 0.0f, 0.0f};
static float accel_lin_world[3] = {0.0f, 0.0f, 0.0f};

// Velocity + position
static float vel_world[3] = {0.0f, 0.0f, 0.0f};
static float pos_world[3] = {0.0f, 0.0f, 0.0f};

// Optional mag calibration placeholders
static float mag_offset[3] = {0.0f, 0.0f, 0.0f};
static float mag_scale[3] = {1.0f, 1.0f, 1.0f};

static constexpr float ACC_DEADBAND_MSS = 0.08f;
static constexpr float VEL_DAMPING = 0.995f;

// Rotate vector by quaternion: out = q * v * q_conj
static void rotateVectorByQuat(const float q[4], const float v[3], float out[3])
{
  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  const float tx = 2.0f * (qy * v[2] - qz * v[1]);
  const float ty = 2.0f * (qz * v[0] - qx * v[2]);
  const float tz = 2.0f * (qx * v[1] - qy * v[0]);

  out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
  out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
  out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

// Rotate vector from world to body using q_conj
static void rotateVectorWorldToBody(const float q[4], const float v[3], float out[3])
{
  float qc[4] = {q[0], -q[1], -q[2], -q[3]};
  rotateVectorByQuat(qc, v, out);
}

static void updateEulerFromQuat()
{
  const float qw = quat[0];
  const float qx = quat[1];
  const float qy = quat[2];
  const float qz = quat[3];

  const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  const float sinp = 2.0f * (qw * qy - qz * qx);
  if (fabsf(sinp) >= 1.0f)
    pitch_deg = copysignf(90.0f, sinp);
  else
    pitch_deg = asinf(sinp) * 180.0f / PI;

  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

static void applyDeadband(float v[3], float threshold)
{
  for (int i = 0; i < 3; ++i)
  {
    if (fabsf(v[i]) < threshold)
      v[i] = 0.0f;
  }
}

static bool readRawIMU()
{
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;
  sensors_event_t temp_event;

  icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

  // Adafruit returns accel in m/s^2
  raw_ax_mss = accel_event.acceleration.x;
  raw_ay_mss = accel_event.acceleration.y;
  raw_az_mss = accel_event.acceleration.z;

  // Adafruit returns gyro in rad/s, convert to deg/s for Madgwick
  raw_gx_dps = gyro_event.gyro.x * RAD_TO_DEG_F;
  raw_gy_dps = gyro_event.gyro.y * RAD_TO_DEG_F;
  raw_gz_dps = gyro_event.gyro.z * RAD_TO_DEG_F;

  // Adafruit returns magnetometer in uT
  raw_mx_uT = (mag_event.magnetic.x - mag_offset[0]) * mag_scale[0];
  raw_my_uT = (mag_event.magnetic.y - mag_offset[1]) * mag_scale[1];
  raw_mz_uT = (mag_event.magnetic.z - mag_offset[2]) * mag_scale[2];

  return true;
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

  // Optional range setup
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  // Optional rate divider settings can be added later if needed

  filter.begin(100.0f);
  last_update_us = micros();
  imu_ok = true;

  Serial.println("ICM-20948 ready");
  return true;
}

void imu_update()
{
  if (!imu_ok)
    return;

  if (!readRawIMU())
    return;

  unsigned long now_us = micros();
  float dt = (now_us - last_update_us) * 1e-6f;
  last_update_us = now_us;

  if (dt <= 0.0f || dt > 0.1f)
    dt = 0.01f;

  // Some Madgwick Arduino libs use begin(sampleRateHz) as the timestep source
  filter.begin(1.0f / dt);

  const float ax_g = raw_ax_mss / G_MSS;
  const float ay_g = raw_ay_mss / G_MSS;
  const float az_g = raw_az_mss / G_MSS;

  filter.update(
      raw_gx_dps, raw_gy_dps, raw_gz_dps,
      ax_g, ay_g, az_g,
      raw_mx_uT, raw_my_uT, raw_mz_uT);

  quat[0] = filter.q0;
  quat[1] = filter.q1;
  quat[2] = filter.q2;
  quat[3] = filter.q3;

  updateEulerFromQuat();

  // Gravity vector in world frame
  const float gravity_world[3] = {0.0f, 0.0f, G_MSS};

  // Convert gravity into body frame
  float gravity_body[3];
  rotateVectorWorldToBody(quat, gravity_world, gravity_body);

  // Remove gravity from raw accel in body frame
  accel_lin_body[0] = raw_ax_mss - gravity_body[0];
  accel_lin_body[1] = raw_ay_mss - gravity_body[1];
  accel_lin_body[2] = raw_az_mss - gravity_body[2];
  applyDeadband(accel_lin_body, ACC_DEADBAND_MSS);

  // Rotate linear accel into world frame
  rotateVectorByQuat(quat, accel_lin_body, accel_lin_world);
  applyDeadband(accel_lin_world, ACC_DEADBAND_MSS);

  // Integrate to velocity and position
  for (int i = 0; i < 3; ++i)
  {
    vel_world[i] += accel_lin_world[i] * dt;
    vel_world[i] *= VEL_DAMPING;

    if (fabsf(accel_lin_world[i]) < ACC_DEADBAND_MSS)
      vel_world[i] *= 0.98f;

    pos_world[i] += vel_world[i] * dt;
  }
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
  ax = accel_lin_body[0];
  ay = accel_lin_body[1];
  az = accel_lin_body[2];
}

void imu_getLinearAccelWorld(float &ax, float &ay, float &az)
{
  ax = accel_lin_world[0];
  ay = accel_lin_world[1];
  az = accel_lin_world[2];
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
  x = pos_world[0];
  y = pos_world[1];
  z = pos_world[2];
}

void imu_getVelocity(float &x, float &y, float &z)
{
  x = vel_world[0];
  y = vel_world[1];
  z = vel_world[2];
}

void imu_resetPosition()
{
  vel_world[0] = vel_world[1] = vel_world[2] = 0.0f;
  pos_world[0] = pos_world[1] = pos_world[2] = 0.0f;
}

void imu_printDebug()
{
  const unsigned long now = millis();
  if (now - last_debug_ms < 100)
    return;
  last_debug_ms = now;

  float raw_ax, raw_ay, raw_az;
  float raw_gx, raw_gy, raw_gz;
  float raw_mx, raw_my, raw_mz;
  float lin_ax, lin_ay, lin_az;
  float world_ax, world_ay, world_az;
  float px, py, pz;
  float vx, vy, vz;

  imu_getRawAccel(raw_ax, raw_ay, raw_az);
  imu_getRawGyro(raw_gx, raw_gy, raw_gz);
  imu_getRawMag(raw_mx, raw_my, raw_mz);
  imu_getLinearAccel(lin_ax, lin_ay, lin_az);
  imu_getLinearAccelWorld(world_ax, world_ay, world_az);
  imu_getPosition(px, py, pz);
  imu_getVelocity(vx, vy, vz);

  Serial.println("=== IMU ===");

  Serial.print("RAW ACC m/s^2: ");
  Serial.print(raw_ax, 4); Serial.print(", ");
  Serial.print(raw_ay, 4); Serial.print(", ");
  Serial.println(raw_az, 4);

  Serial.print("RAW GYRO deg/s: ");
  Serial.print(raw_gx, 4); Serial.print(", ");
  Serial.print(raw_gy, 4); Serial.print(", ");
  Serial.println(raw_gz, 4);

  Serial.print("RAW MAG uT: ");
  Serial.print(raw_mx, 4); Serial.print(", ");
  Serial.print(raw_my, 4); Serial.print(", ");
  Serial.println(raw_mz, 4);

  Serial.print("LIN ACC BODY m/s^2: ");
  Serial.print(lin_ax, 4); Serial.print(", ");
  Serial.print(lin_ay, 4); Serial.print(", ");
  Serial.println(lin_az, 4);

  Serial.print("LIN ACC WORLD m/s^2: ");
  Serial.print(world_ax, 4); Serial.print(", ");
  Serial.print(world_ay, 4); Serial.print(", ");
  Serial.println(world_az, 4);

  Serial.print("RPY deg: ");
  Serial.print(roll_deg, 2); Serial.print(", ");
  Serial.print(pitch_deg, 2); Serial.print(", ");
  Serial.println(yaw_deg, 2);

  Serial.print("POS m: ");
  Serial.print(px, 4); Serial.print(", ");
  Serial.print(py, 4); Serial.print(", ");
  Serial.println(pz, 4);

  Serial.print("VEL m/s: ");
  Serial.print(vx, 4); Serial.print(", ");
  Serial.print(vy, 4); Serial.print(", ");
  Serial.println(vz, 4);
}