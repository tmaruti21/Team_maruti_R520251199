/*
 * ============================================
 * BEETLE ROBOT - VL53L0X ToF SENSOR TEST
 * ============================================
 * 5 VL53L0X Time-of-Flight distance sensors on I2C
 * Each sensor is enabled sequentially via XSHUT pins
 * and assigned a unique I2C address.
 *
 * Output format (CSV):
 * t_ms,tof1_mm,tof2_mm,tof3_mm,tof4_mm,tof5_mm,tof1_status,tof2_status,tof3_status,tof4_status,tof5_status
 *
 * Status: 0 = valid, 1 = sigma fail, 2 = signal fail,
 *         3 = min range fail, 4 = phase fail (out of range)
 *
 * Wiring:
 *   SDA  -> GPIO 21
 *   SCL  -> GPIO 22
 *   XSHUT1 -> GPIO 16
 *   XSHUT2 -> GPIO 17
 *   XSHUT3 -> GPIO 2
 *   XSHUT4 -> GPIO 15
 *   XSHUT5 -> GPIO 12
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ================= SERIAL ====================
#define BAUDRATE 115200
#define SEND_INTERVAL 100  // ms (~10 Hz)

// ================= I2C =======================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= TOF XSHUT PINS ============
#define TOF1_XSHUT 16
#define TOF2_XSHUT 17
#define TOF3_XSHUT 2
#define TOF4_XSHUT 15
#define TOF5_XSHUT 12

#define NUM_TOF 5

const uint8_t tofXshut[NUM_TOF] = {
  TOF1_XSHUT, TOF2_XSHUT, TOF3_XSHUT, TOF4_XSHUT, TOF5_XSHUT
};

// Unique I2C addresses for each sensor (default is 0x29)
const uint8_t tofAddr[NUM_TOF] = {
  0x30, 0x31, 0x32, 0x33, 0x34
};

Adafruit_VL53L0X tof[NUM_TOF];
bool tofOK[NUM_TOF];

// ================= SETUP =====================
void setup() {
  Serial.begin(BAUDRATE);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // --- Initialize all ToF sensors ---
  // Step 1: Shut down ALL sensors
  for (int i = 0; i < NUM_TOF; i++) {
    pinMode(tofXshut[i], OUTPUT);
    digitalWrite(tofXshut[i], LOW);
  }
  delay(100);

  // Step 2: Enable one at a time and assign unique address
  for (int i = 0; i < NUM_TOF; i++) {
    digitalWrite(tofXshut[i], HIGH);
    delay(50);  // Let sensor boot

    if (tof[i].begin(tofAddr[i])) {
      tofOK[i] = true;
      Serial.print("ToF ");
      Serial.print(i + 1);
      Serial.print(" OK at 0x");
      Serial.println(tofAddr[i], HEX);
    } else {
      tofOK[i] = false;
      Serial.print("ToF ");
      Serial.print(i + 1);
      Serial.println(" FAILED");
    }
  }

  // Print startup summary
  int okCount = 0;
  for (int i = 0; i < NUM_TOF; i++) {
    if (tofOK[i]) okCount++;
  }
  Serial.print("ToF sensors initialized: ");
  Serial.print(okCount);
  Serial.print("/");
  Serial.println(NUM_TOF);

  // CSV Header
  Serial.println("t_ms,tof1_mm,tof2_mm,tof3_mm,tof4_mm,tof5_mm,tof1_status,tof2_status,tof3_status,tof4_status,tof5_status");
}

// ================= LOOP ======================
void loop() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();

  if (now - lastSend < SEND_INTERVAL) return;
  lastSend = now;

  int dist[NUM_TOF];
  uint8_t status[NUM_TOF];

  // Read each ToF sensor
  for (int i = 0; i < NUM_TOF; i++) {
    if (tofOK[i]) {
      VL53L0X_RangingMeasurementData_t measure;
      tof[i].rangingTest(&measure, false);

      status[i] = measure.RangeStatus;

      if (measure.RangeStatus == 4) {
        // Phase failure — out of range
        dist[i] = 8190;
      } else {
        dist[i] = measure.RangeMilliMeter;
      }
    } else {
      dist[i] = -1;   // Sensor not available
      status[i] = 255; // Invalid
    }
  }

  // CSV Output: timestamp, 5 distances, 5 statuses
  Serial.print(now);
  Serial.print(",");

  // Distances
  for (int i = 0; i < NUM_TOF; i++) {
    Serial.print(dist[i]);
    Serial.print(",");
  }

  // Statuses
  for (int i = 0; i < NUM_TOF; i++) {
    Serial.print(status[i]);
    if (i < NUM_TOF - 1) Serial.print(",");
  }

  Serial.println();
}
