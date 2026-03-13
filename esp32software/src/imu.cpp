#include "imu.h"
#include <MadgwickAHRS.h>
#include <Wire.h>

// If you use a specific ICM20948 library, include it here and
// replace the TODO block in readRawIMU() with calls to that library.
// Example line you might add if using SparkFun library:
// #include <SparkFun_ICM-20948_ArduinoLibrary.h>
// static ICM_20948_I2C icm;

static Madgwick filter;
static float accel_lin[3] = {0.0f, 0.0f, 0.0f};
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // q0,q1,q2,q3

// raw readings in SI (expected)
static float raw_ax = 0.0f;
static float raw_ay = 0.0f;
static float raw_az = 9.80665f;
static float raw_gx = 0.0f;
static float raw_gy = 0.0f;
static float raw_gz = 0.0f;

// Simple quaternion rotate helper: out = q * v * q_conj
static void rotateVectorByQuat(const float q[4], const float v[3], float out[3])
{
  // q = [q0, q1, q2, q3] (q0 = w)
  const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  // q * v_quat
  float t0 = -q1 * v[0] - q2 * v[1] - q3 * v[2];
  float t1 =  q0 * v[0] + q2 * v[2] - q3 * v[1];
  float t2 =  q0 * v[1] + q3 * v[0] - q1 * v[2];
  float t3 =  q0 * v[2] + q1 * v[1] - q2 * v[0];
  // result = t * q_conj
  out[0] = -t0 * q1 + t1 * q0 - t2 * q3 + t3 * q2;
  out[1] = -t0 * q2 + t2 * q0 - t3 * q1 + t1 * q3;
  out[2] = -t0 * q3 + t3 * q0 - t1 * q2 + t2 * q1;
}

// compute gravity vector in body frame (m/s^2) from current quaternion
static void computeGravityBody(const float q[4], float out_g[3])
{
  float gw[3] = {0.0f, 0.0f, 1.0f}; // world up unit vector
  rotateVectorByQuat(q, gw, out_g);
  out_g[0] *= 9.80665f;
  out_g[1] *= 9.80665f;
  out_g[2] *= 9.80665f;
}

// TODO: Replace the body of this function with your actual ICM20948 library reads.
// The function must fill raw_ax/raw_ay/raw_az in m/s^2 and raw_gx/raw_gy/raw_gz in deg/s
// Example conversions you might need:
// - If your IMU provides accel in g, multiply by 9.80665
// - If your IMU provides gyro in rad/s and Madgwick expects deg/s, convert rad->deg
static void readRawIMU()
{
  // ===  PLACEHOLDER READ ===
  // The code below is a safe placeholder so you can compile and test pairing and radio.
  // Replace the 3 lines below with your ICM20948 calls.

  // Example pseudo code (replace with actual lib calls):
  // icm.readSensor();
  // raw_ax = icm.getAccelX_mss();
  // raw_ay = icm.getAccelY_mss();
  // raw_az = icm.getAccelZ_mss();
  // raw_gx = icm.getGyroX_dps();
  // raw_gy = icm.getGyroY_dps();
  // raw_gz = icm.getGyroZ_dps();

  // For now keep device stationary values so gravity removal shows sensible output
  raw_ax = 0.0f;
  raw_ay = 0.0f;
  raw_az = 9.80665f;
  raw_gx = 0.0f;
  raw_gy = 0.0f;
  raw_gz = 0.0f;
}

// initialize IMU and filter
bool imu_begin()
{
  Wire.begin(); // SDA=21 SCL=22 on your wiring
  delay(50);

  // If using a library, init it here. Example:
  // if (icm.begin() != ICM_20948_Stat_Ok) { Serial.println("ICM init failed"); return false; }

  // Initialize Madgwick filter. Parameter is sample rate in Hz.
  // Tune sample rate to your actual IMU update frequency.
  filter.begin(200.0f);

  // optional filter tuning:
  // filter.setBeta(0.1f); // if your Madgwick library exposes it

  Serial.println("IMU module init done (placeholder). Replace readRawIMU() with your ICM20948 reads.");
  return true;
}

// call this in your main loop at the IMU update rate
void imu_update()
{
  // 1) read raw IMU
  readRawIMU();

  // 2) update Madgwick.
  // Many Madgwick ports expect gyro in deg/s and accel in g.
  // Here we convert accel from m/s^2 to g before calling updateIMU.
  float ax_g = raw_ax / 9.80665f;
  float ay_g = raw_ay / 9.80665f;
  float az_g = raw_az / 9.80665f;

  // Call updateIMU depending on library signature.
  // This matches many Madgwick Arduino ports: updateIMU(gx,gy,gz,ax,ay,az)
  filter.updateIMU(raw_gx, raw_gy, raw_gz, ax_g, ay_g, az_g);

  // store quaternion
  quat[0] = filter.q0;
  quat[1] = filter.q1;
  quat[2] = filter.q2;
  quat[3] = filter.q3;

  // 3) compute gravity vector in body frame (m/s^2) and subtract from raw accel
  float gravity[3];
  computeGravityBody(quat, gravity);

  accel_lin[0] = raw_ax - gravity[0];
  accel_lin[1] = raw_ay - gravity[1];
  accel_lin[2] = raw_az - gravity[2];

  // now accel_lin[] holds approximate linear acceleration with gravity removed
}

void imu_getLinearAccel(float &ax, float &ay, float &az)
{
  ax = accel_lin[0];
  ay = accel_lin[1];
  az = accel_lin[2];
}

void imu_getQuat(float &q0, float &q1, float &q2, float &q3)
{
  q0 = quat[0];
  q1 = quat[1];
  q2 = quat[2];
  q3 = quat[3];
}