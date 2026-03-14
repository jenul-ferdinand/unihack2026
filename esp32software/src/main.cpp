// main.cpp
// Read ICM-20948 over I2C on ESP32 and print JSON lines to Serial.
// Wiring (as provided):
// ICM20948 SDA -> GPIO21
// ICM20948 SCL -> GPIO22
// ICM20948 VCC -> 3.3V
// ICM20948 GND -> GND
//
// XC4508 (nRF24L01 module) wiring noted but not used by this sketch:
// SCK -> GPIO18, MOSI -> GPIO23, MISO -> GPIO19, CSN -> GPIO5, CE -> GPIO4
//
// Requires libraries:
//   Adafruit ICM20X (Adafruit_ICM20X) and Adafruit Sensor

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 icm; // Adafruit ICM20X wrapper for ICM-20948

// I2C pins you specified
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

// Output interval (ms)
constexpr unsigned long SAMPLE_INTERVAL_MS = 100; // 10 Hz

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(5);

  // Explicitly start I2C on the ESP32 pins you provided
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(10);

  Serial.println(F("Starting ICM-20948 (I2C) example..."));

  if (!icm.begin_I2C(0x68))
  {
    Serial.println(F("ERROR: ICM20948 not found via I2C. Check wiring and power."));
    // Halt - comment out if you prefer retrying
    while (1)
    {
      delay(1000);
    }
  }

  Serial.println(F("ICM-20948 detected."));

  // Optional: set ranges or data rates here if desired, example:
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  // icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  // Give sensor time to stabilize
  delay(100);
}

void loop()
{
  static unsigned long lastMillis = 0;
  unsigned long now = millis();
  if (now - lastMillis < SAMPLE_INTERVAL_MS)
    return;
  lastMillis = now;

  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;
  sensors_event_t temp_event;

  // Read normalized sensor events (uses Adafruit unified sensor types)
  icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

  // Build JSON string manually. Use String for simplicity.
  // Values: accel in m/s^2, gyro in rad/s, mag in uT, temp in C.

  String json = "{";
  json += "\"timestamp\":";
  json += String(now);
  json += ",";

  json += "\"temp_C\":";
  json += String(temp_event.temperature, 3);
  json += ",";

  json += "\"accel\":{";
  json += "\"x\":" + String(accel_event.acceleration.x, 6) + ",";
  json += "\"y\":" + String(accel_event.acceleration.y, 6) + ",";
  json += "\"z\":" + String(accel_event.acceleration.z, 6);
  json += "},";

  json += "\"gyro\":{";
  json += "\"x\":" + String(gyro_event.gyro.x, 6) + ",";
  json += "\"y\":" + String(gyro_event.gyro.y, 6) + ",";
  json += "\"z\":" + String(gyro_event.gyro.z, 6);
  json += "},";

  json += "\"mag\":{";
  json += "\"x\":" + String(mag_event.magnetic.x, 6) + ",";
  json += "\"y\":" + String(mag_event.magnetic.y, 6) + ",";
  json += "\"z\":" + String(mag_event.magnetic.z, 6);
  json += "}";

  json += "}";

  // Print one JSON object per line for easy parsing
  Serial.println(json);
}
