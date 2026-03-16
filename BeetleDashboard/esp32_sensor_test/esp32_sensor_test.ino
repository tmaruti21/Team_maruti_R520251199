/*
 * ============================================
 * BEETLE ROBOT - ALL SENSORS COMBINED
 * ============================================
 * Sensors:
 *   - 1x MPU6050 (I2C, raw register access)
 *   - 6x Quadrature Encoders (3 left + 3 right, averaged)
 *   - 4x VL53L0X ToF Sensors (I2C, XSHUT addressing)
 *
 * CSV Output (14 columns):
 *   t_ms, ax, ay, az, gx, gy, gz, temp,
 *   left_ticks, right_ticks,
 *   tof1, tof2, tof3, tof4
 *
 * Units:
 *   ax,ay,az       = m/s²
 *   gx,gy,gz       = rad/s
 *   temp           = °C (MPU6050 built-in)
 *   left/right     = average delta ticks per interval
 *   tof1-4         = distance in mm
 *
 * Update rate: ~10 Hz (100 ms)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =================== SERIAL ===================
#define BAUDRATE 115200
#define SEND_INTERVAL 100  // ms (~10 Hz)

// =================== I2C =======================
#define SDA_PIN 21
#define SCL_PIN 22

// =================== MPU6050 ===================
#define MPU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

#define ACCEL_LSB_PER_G   16384.0
#define GYRO_LSB_PER_DPS    131.0

const float G_TO_MS2    = 9.80665;
const float DPS_TO_RADS = 0.017453292519943295;

bool mpuOK = false;

// =================== ENCODERS (6) ==============
// Left side:  ENC1 (front), ENC2 (mid), ENC3 (rear)
// Right side: ENC4 (front), ENC5 (mid), ENC6 (rear)
#define NUM_ENC 6

#define ENC1_A 32
#define ENC2_A 33
#define ENC3_A 25
#define ENC4_A 26
#define ENC5_A 27
#define ENC6_A 14

#define ENC1_B 4
#define ENC2_B 5
#define ENC3_B 18
#define ENC4_B 19
#define ENC5_B 23
#define ENC6_B 13

const uint8_t encA[NUM_ENC] = {ENC1_A, ENC2_A, ENC3_A, ENC4_A, ENC5_A, ENC6_A};
const uint8_t encB[NUM_ENC] = {ENC1_B, ENC2_B, ENC3_B, ENC4_B, ENC5_B, ENC6_B};

volatile long encoderCount[NUM_ENC] = {0};

void IRAM_ATTR encISR0() { encoderCount[0] += digitalRead(encA[0]) == digitalRead(encB[0]) ? 1 : -1; }
void IRAM_ATTR encISR1() { encoderCount[1] += digitalRead(encA[1]) == digitalRead(encB[1]) ? 1 : -1; }
void IRAM_ATTR encISR2() { encoderCount[2] += digitalRead(encA[2]) == digitalRead(encB[2]) ? 1 : -1; }
void IRAM_ATTR encISR3() { encoderCount[3] += digitalRead(encA[3]) == digitalRead(encB[3]) ? 1 : -1; }
void IRAM_ATTR encISR4() { encoderCount[4] += digitalRead(encA[4]) == digitalRead(encB[4]) ? 1 : -1; }
void IRAM_ATTR encISR5() { encoderCount[5] += digitalRead(encA[5]) == digitalRead(encB[5]) ? 1 : -1; }

void (*encISR[NUM_ENC])() = {encISR0, encISR1, encISR2, encISR3, encISR4, encISR5};

// =================== TOF (4) ===================
#define NUM_TOF 4

#define TOF1_XSHUT 16
#define TOF2_XSHUT 17
#define TOF3_XSHUT 2
#define TOF4_XSHUT 15

const uint8_t tofXshut[NUM_TOF] = {
  TOF1_XSHUT, TOF2_XSHUT, TOF3_XSHUT, TOF4_XSHUT
};

const uint8_t tofAddr[NUM_TOF] = {
  0x30, 0x31, 0x32, 0x33
};

Adafruit_VL53L0X tof[NUM_TOF];
bool tofOK[NUM_TOF];

// =================== TIMING ====================
unsigned long lastUpdateTime = 0;

// =================== MPU HELPERS ===============

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, count, (uint8_t)true);
  for (int i = 0; i < count; i++) dest[i] = Wire.read();
}

// =================== SETUP =====================

void setup() {
  Serial.begin(BAUDRATE);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // ----------- MPU6050 INIT ----------
  mpuWriteReg(PWR_MGMT_1, 0x00);
  delay(100);
  mpuWriteReg(ACCEL_CONFIG, 0x00);  // ±2g
  mpuWriteReg(GYRO_CONFIG,  0x00);  // ±250 dps
  delay(50);

  uint8_t whoAmI = 0;
  mpuReadBytes(0x75, 1, &whoAmI);
  mpuOK = (whoAmI == 0x68);
  Serial.print("MPU6050: ");
  Serial.println(mpuOK ? "OK" : "FAILED");

  // ----------- ENCODERS INIT (6) -----
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA[i]), encISR[i], CHANGE);
  }
  Serial.println("Encoders (6): OK");

  // ----------- TOF INIT (4) ----------
  for (int i = 0; i < NUM_TOF; i++) {
    pinMode(tofXshut[i], OUTPUT);
    digitalWrite(tofXshut[i], LOW);
  }
  delay(100);

  int tofCount = 0;
  for (int i = 0; i < NUM_TOF; i++) {
    digitalWrite(tofXshut[i], HIGH);
    delay(50);

    if (tof[i].begin(tofAddr[i])) {
      tofOK[i] = true;
      tofCount++;
      Serial.print("ToF "); Serial.print(i + 1);
      Serial.print(": OK (0x"); Serial.print(tofAddr[i], HEX); Serial.println(")");
    } else {
      tofOK[i] = false;
      Serial.print("ToF "); Serial.print(i + 1); Serial.println(": FAILED");
    }
  }
  Serial.print("ToF sensors: ");
  Serial.print(tofCount); Serial.print("/"); Serial.println(NUM_TOF);

  // ----------- READY -----------------
  Serial.println();
  Serial.println("=== ALL SENSORS READY ===");
  Serial.println();

  // CSV Header
  Serial.println("t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4");

  lastUpdateTime = millis();
}

// =================== LOOP ======================

void loop() {
  unsigned long now = millis();
  if (now - lastUpdateTime < SEND_INTERVAL) return;
  lastUpdateTime = now;

  // --------- 1. ENCODERS (snapshot + reset) -----
  long enc[NUM_ENC];
  noInterrupts();
  for (int i = 0; i < NUM_ENC; i++) {
    enc[i] = encoderCount[i];
    encoderCount[i] = 0;
  }
  interrupts();

  // Average left encoders (enc1, enc2, enc3)
  float leftTicks = (enc[0] + enc[1] + enc[2]) / 3.0;
  // Average right encoders (enc4, enc5, enc6)
  float rightTicks = (enc[3] + enc[4] + enc[5]) / 3.0;

  // --------- 2. MPU6050 -------------------------
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float tempC = 0;

  if (mpuOK) {
    uint8_t buf[14];
    mpuReadBytes(ACCEL_XOUT_H, 14, buf);

    int16_t ax_raw  = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t ay_raw  = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t az_raw  = (int16_t)(buf[4]  << 8 | buf[5]);
    int16_t tmp_raw = (int16_t)(buf[6]  << 8 | buf[7]);
    int16_t gx_raw  = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t gy_raw  = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t gz_raw  = (int16_t)(buf[12] << 8 | buf[13]);

    ax = (ax_raw / ACCEL_LSB_PER_G) * G_TO_MS2;
    ay = (ay_raw / ACCEL_LSB_PER_G) * G_TO_MS2;
    az = (az_raw / ACCEL_LSB_PER_G) * G_TO_MS2;

    gx = (gx_raw / GYRO_LSB_PER_DPS) * DPS_TO_RADS;
    gy = (gy_raw / GYRO_LSB_PER_DPS) * DPS_TO_RADS;
    gz = (gz_raw / GYRO_LSB_PER_DPS) * DPS_TO_RADS;

    tempC = (tmp_raw / 333.87) + 21.0;
  }

  // --------- 3. TOF SENSORS (4) -----------------
  int tofDist[NUM_TOF];
  for (int i = 0; i < NUM_TOF; i++) {
    if (tofOK[i]) {
      VL53L0X_RangingMeasurementData_t measure;
      tof[i].rangingTest(&measure, false);
      tofDist[i] = (measure.RangeStatus == 4) ? 8190 : measure.RangeMilliMeter;
    } else {
      tofDist[i] = -1;
    }
  }

  // --------- 4. CSV OUTPUT -----------------------
  Serial.print("DATA:");

  // t_ms
  Serial.print(now); Serial.print(",");

  // MPU: ax, ay, az, gx, gy, gz, temp
  Serial.print(ax,  4); Serial.print(",");
  Serial.print(ay,  4); Serial.print(",");
  Serial.print(az,  4); Serial.print(",");
  Serial.print(gx,  5); Serial.print(",");
  Serial.print(gy,  5); Serial.print(",");
  Serial.print(gz,  5); Serial.print(",");
  Serial.print(tempC, 2); Serial.print(",");

  // Averaged encoder ticks
  Serial.print(leftTicks, 2); Serial.print(",");
  Serial.print(rightTicks, 2); Serial.print(",");

  // ToF distances (4 sensors)
  for (int i = 0; i < NUM_TOF; i++) {
    Serial.print(tofDist[i]);
    if (i < NUM_TOF - 1) Serial.print(",");
  }

  Serial.println();
}
