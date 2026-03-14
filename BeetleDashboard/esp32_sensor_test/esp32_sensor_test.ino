/*
 * ============================================
 * BEETLE ROBOT - ESP32 SENSOR NODE (FINAL)
 * ============================================
 * Sensors:
 *  - MPU6050 (I2C)
 *  - 6 Encoders (Quadrature)
 *  - 5 VL53L0X ToF Sensors
 *
 * Output format:
 * ax,ay,az,gx,gy,gz,e1,e2,e3,e4,e5,e6,t1,t2,t3,t4,t5
 */
#include <ard
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>

// ================= SERIAL ====================
#define BAUDRATE 115200
#define SEND_INTERVAL 100

// ================= I2C =======================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= ENCODERS ==================
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

const uint8_t encA[6] = {ENC1_A, ENC2_A, ENC3_A, ENC4_A, ENC5_A, ENC6_A};
const uint8_t encB[6] = {ENC1_B, ENC2_B, ENC3_B, ENC4_B, ENC5_B, ENC6_B};

volatile long encoderCount[6] = {0};

// ================= TOF =======================
#define TOF1_XSHUT 16
#define TOF2_XSHUT 17
#define TOF3_XSHUT 2
#define TOF4_XSHUT 15
#define TOF5_XSHUT 12

const uint8_t tofXshut[5] = {
  TOF1_XSHUT, TOF2_XSHUT, TOF3_XSHUT, TOF4_XSHUT, TOF5_XSHUT
};

const uint8_t tofAddr[5] = {
  0x30, 0x31, 0x32, 0x33, 0x34
};

Adafruit_VL53L0X tof[5];
bool tofOK[5];

// ================= MPU =======================
Adafruit_MPU6050 mpu;
bool mpuOK = false;

// ================= ISR =======================
void IRAM_ATTR encISR0(){ encoderCount[0] += digitalRead(encA[0]) == digitalRead(encB[0]) ? 1 : -1; }
void IRAM_ATTR encISR1(){ encoderCount[1] += digitalRead(encA[1]) == digitalRead(encB[1]) ? 1 : -1; }
void IRAM_ATTR encISR2(){ encoderCount[2] += digitalRead(encA[2]) == digitalRead(encB[2]) ? 1 : -1; }
void IRAM_ATTR encISR3(){ encoderCount[3] += digitalRead(encA[3]) == digitalRead(encB[3]) ? 1 : -1; }
void IRAM_ATTR encISR4(){ encoderCount[4] += digitalRead(encA[4]) == digitalRead(encB[4]) ? 1 : -1; }
void IRAM_ATTR encISR5(){ encoderCount[5] += digitalRead(encA[5]) == digitalRead(encB[5]) ? 1 : -1; }

void (*isr[6])() = {encISR0, encISR1, encISR2, encISR3, encISR4, encISR5};

// ================= SETUP =====================
void setup() {
  Serial.begin(BAUDRATE);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // MPU6050
  mpuOK = mpu.begin();
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Encoders
  for (int i = 0; i < 6; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA[i]), isr[i], CHANGE);
  }

  // ToF setup
  for (int i = 0; i < 5; i++) {
    pinMode(tofXshut[i], OUTPUT);
    digitalWrite(tofXshut[i], LOW);
  }
  delay(100);

  for (int i = 0; i < 5; i++) {
    digitalWrite(tofXshut[i], HIGH);
    delay(50);
    if (tof[i].begin()) {
      tof[i].setAddress(tofAddr[i]);
      tofOK[i] = true;
    } else {
      tofOK[i] = false;
    }
  }

  Serial.println("READY");
}

// ================= LOOP ======================
void loop() {
  static unsigned long last = 0;
  if (millis() - last < SEND_INTERVAL) return;
  last = millis();

  // MPU
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (mpuOK) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x * 57.2958;
    gy = g.gyro.y * 57.2958;
    gz = g.gyro.z * 57.2958;
  }

  // Encoders
  long enc[6];
  noInterrupts();
    for (int i = 0; i < 6; i++) {
      enc[i] = encoderCount[i];
    }
  interrupts();

  // ToF
  int dist[5];
  for (int i = 0; i < 5; i++) {
    if (tofOK[i]) {
      VL53L0X_RangingMeasurementData_t m;
      tof[i].rangingTest(&m, false);
      dist[i] = (m.RangeStatus == 4) ? 8190 : m.RangeMilliMeter;
    } else dist[i] = 0;
  }

  // Serial CSV
  // Serial CSV
  Serial.print("DATA:");
  Serial.print(ax,2); Serial.print(",");
  Serial.print(ay,2); Serial.print(",");
  Serial.print(az,2); Serial.print(",");
  Serial.print(gx,2); Serial.print(",");
  Serial.print(gy,2); Serial.print(",");
  Serial.print(gz,2); Serial.print(",");

  for (int i=0;i<6;i++){ Serial.print(enc[i]); Serial.print(","); }
  
  // ToF (7 sensors expected by dashboard)
  for (int i=0;i<7;i++){
    if (i < 5) Serial.print(dist[i]);
    else Serial.print(0); // Pad missing sensors
    
    if (i<6) Serial.print(",");
  }
  Serial.println();
}