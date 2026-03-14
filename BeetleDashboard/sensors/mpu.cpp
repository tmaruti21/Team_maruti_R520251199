#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

// Registers
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

// Sensitivity (for ±2g and ±250 dps)
#define ACCEL_LSB_PER_G   16384.0
#define GYRO_LSB_PER_DPS    131.0

const float G_TO_MS2 = 9.80665;
const float DPS_TO_RADS = 0.017453292519943295; // pi/180

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void readBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count, true);
  for (int i = 0; i < count; i++) dest[i] = Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Wake up
  writeReg(PWR_MGMT_1, 0x00);
  delay(100);

  // Force known ranges: ±2g and ±250 dps
  writeReg(ACCEL_CONFIG, 0x00);
  writeReg(GYRO_CONFIG,  0x00);

  delay(50);

  // Header (optional)
  Serial.println("t_ms,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,temp_C");
}

void loop() {
  uint8_t buf[14];
  readBytes(ACCEL_XOUT_H, 14, buf);

  int16_t ax_raw = (int16_t)(buf[0] << 8 | buf[1]);
  int16_t ay_raw = (int16_t)(buf[2] << 8 | buf[3]);
  int16_t az_raw = (int16_t)(buf[4] << 8 | buf[5]);

  int16_t temp_raw = (int16_t)(buf[6] << 8 | buf[7]);

  int16_t gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
  int16_t gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
  int16_t gz_raw = (int16_t)(buf[12] << 8 | buf[13]);

  // Convert raw -> g
  float ax_g = ax_raw / ACCEL_LSB_PER_G;
  float ay_g = ay_raw / ACCEL_LSB_PER_G;
  float az_g = az_raw / ACCEL_LSB_PER_G;

  // g -> m/s^2 (ROS expects m/s^2)
  float ax_ms2 = ax_g * G_TO_MS2;
  float ay_ms2 = ay_g * G_TO_MS2;
  float az_ms2 = az_g * G_TO_MS2;

  // raw -> deg/s
  float gx_dps = gx_raw / GYRO_LSB_PER_DPS;
  float gy_dps = gy_raw / GYRO_LSB_PER_DPS;
  float gz_dps = gz_raw / GYRO_LSB_PER_DPS;

  // deg/s -> rad/s (ROS expects rad/s)
  float gx_rads = gx_dps * DPS_TO_RADS;
  float gy_rads = gy_dps * DPS_TO_RADS;
  float gz_rads = gz_dps * DPS_TO_RADS;

  // Temperature (optional)
  float tempC = (temp_raw / 333.87) + 21.0;

  // CSV: timestamp + SI units
  Serial.print(millis()); Serial.print(",");
  Serial.print(ax_ms2, 4); Serial.print(",");
  Serial.print(ay_ms2, 4); Serial.print(",");
  Serial.print(az_ms2, 4); Serial.print(",");
  Serial.print(gx_rads, 5); Serial.print(",");
  Serial.print(gy_rads, 5); Serial.print(",");
  Serial.print(gz_rads, 5); Serial.print(",");
  Serial.println(tempC, 2);

  delay(10); // ~100 Hz (good for IMU)
}