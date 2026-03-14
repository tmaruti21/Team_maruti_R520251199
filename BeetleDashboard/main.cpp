/*
 * ============================================
 * BEETLE ROBOT - ROBUST SENSOR NODE
 * ============================================
 * Sensors:
 *   MPU6050 (optional)
 *   6 Encoders
 *   4 VL53L0X (optional)
 *
 * Missing sensors output: null
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ================= SERIAL ===================
#define BAUDRATE 115200
#define SEND_INTERVAL 100

// ================= I2C ======================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= MPU6050 ==================
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

#define ACCEL_LSB_PER_G 16384.0
#define GYRO_LSB_PER_DPS 131.0
  
const float G_TO_MS2 = 9.80665;
const float DPS_TO_RADS = 0.0174532925;

bool mpuOK = false;

// ================= ENCODERS =================
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

const uint8_t encA[NUM_ENC] = {32,33,25,26,27,14};
const uint8_t encB[NUM_ENC] = {4,5,18,19,23,13};

volatile long encoderCount[NUM_ENC] = {0};

void IRAM_ATTR encISR0(){ encoderCount[0]+=digitalRead(encA[0])==digitalRead(encB[0])?1:-1; }
void IRAM_ATTR encISR1(){ encoderCount[1]+=digitalRead(encA[1])==digitalRead(encB[1])?1:-1; }
void IRAM_ATTR encISR2(){ encoderCount[2]+=digitalRead(encA[2])==digitalRead(encB[2])?1:-1; }
void IRAM_ATTR encISR3(){ encoderCount[3]+=digitalRead(encA[3])==digitalRead(encB[3])?1:-1; }
void IRAM_ATTR encISR4(){ encoderCount[4]+=digitalRead(encA[4])==digitalRead(encB[4])?1:-1; }
void IRAM_ATTR encISR5(){ encoderCount[5]+=digitalRead(encA[5])==digitalRead(encB[5])?1:-1; }

void (*encISR[NUM_ENC])()={encISR0,encISR1,encISR2,encISR3,encISR4,encISR5};

// ================= TOF ======================
#define NUM_TOF 4

#define TOF1_XSHUT 16
#define TOF2_XSHUT 17
#define TOF3_XSHUT 2
#define TOF4_XSHUT 15

const uint8_t tofXshut[NUM_TOF]={16,17,2,15};
const uint8_t tofAddr[NUM_TOF]={0x30,0x31,0x32,0x33};

Adafruit_VL53L0X tof[NUM_TOF];
bool tofOK[NUM_TOF];

// ================= TIME =====================
unsigned long lastUpdate=0;

// ================= I2C CHECK ================
bool i2cDevicePresent(uint8_t addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission()==0);
}

// ================= MPU READ =================
bool readMPU(float &ax,float &ay,float &az,
             float &gx,float &gy,float &gz,
             float &tempC){

  uint8_t buf[14];

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);

  if(Wire.endTransmission(false)!=0) return false;

  if(Wire.requestFrom(MPU_ADDR,14)!=14) return false;

  for(int i=0;i<14;i++) buf[i]=Wire.read();

  int16_t ax_raw=(buf[0]<<8)|buf[1];
  int16_t ay_raw=(buf[2]<<8)|buf[3];
  int16_t az_raw=(buf[4]<<8)|buf[5];
  int16_t tmp_raw=(buf[6]<<8)|buf[7];
  int16_t gx_raw=(buf[8]<<8)|buf[9];
  int16_t gy_raw=(buf[10]<<8)|buf[11];
  int16_t gz_raw=(buf[12]<<8)|buf[13];

  ax=(ax_raw/ACCEL_LSB_PER_G)*G_TO_MS2;
  ay=(ay_raw/ACCEL_LSB_PER_G)*G_TO_MS2;
  az=(az_raw/ACCEL_LSB_PER_G)*G_TO_MS2;

  gx=(gx_raw/GYRO_LSB_PER_DPS)*DPS_TO_RADS;
  gy=(gy_raw/GYRO_LSB_PER_DPS)*DPS_TO_RADS;
  gz=(gz_raw/GYRO_LSB_PER_DPS)*DPS_TO_RADS;

  tempC=(tmp_raw/333.87)+21.0;

  return true;
}

// ================= SETUP ====================
void setup(){

  Serial.begin(BAUDRATE);
  delay(500);

  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000);

  // MPU detection
  mpuOK=i2cDevicePresent(MPU_ADDR);

  // Encoders
  for(int i=0;i<NUM_ENC;i++){
    pinMode(encA[i],INPUT_PULLUP);
    pinMode(encB[i],INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA[i]),encISR[i],CHANGE);
  }

  // ToF reset
  for(int i=0;i<NUM_TOF;i++){
    pinMode(tofXshut[i],OUTPUT);
    digitalWrite(tofXshut[i],LOW);
  }

  delay(100);

  // ToF init
  for(int i=0;i<NUM_TOF;i++){

    digitalWrite(tofXshut[i],HIGH);
    delay(50);

    if(i2cDevicePresent(0x29) && tof[i].begin(tofAddr[i])){
      tofOK[i]=true;
    }else{
      tofOK[i]=false;
    }
  }

  Serial.println("READY");
}

// ================= LOOP =====================
void loop(){

  if(millis()-lastUpdate<SEND_INTERVAL) return;
  lastUpdate=millis();

  long enc[NUM_ENC];

  noInterrupts();
  for(int i=0;i<NUM_ENC;i++){
    enc[i]=encoderCount[i];
    encoderCount[i]=0;
  }
  interrupts();

  float leftTicks=(enc[0]+enc[1]+enc[2])/3.0;
  float rightTicks=(enc[3]+enc[4]+enc[5])/3.0;

  float ax,ay,az,gx,gy,gz,tempC;
  bool mpuReadOK=false;

  if(mpuOK)
    mpuReadOK=readMPU(ax,ay,az,gx,gy,gz,tempC);

  int dist[NUM_TOF];

  for(int i=0;i<NUM_TOF;i++){

    if(tofOK[i]){
      VL53L0X_RangingMeasurementData_t m;
      tof[i].rangingTest(&m,false);

      if(m.RangeStatus!=4)
        dist[i]=m.RangeMilliMeter;
      else
        dist[i]=8190;

    }else{
      dist[i]=-1;
    }
  }

  // ===== CSV =====

  Serial.print("DATA:");
  Serial.print(millis()); Serial.print(",");

  if(mpuReadOK){
    Serial.print(ax,4); Serial.print(",");
    Serial.print(ay,4); Serial.print(",");
    Serial.print(az,4); Serial.print(",");
    Serial.print(gx,5); Serial.print(",");
    Serial.print(gy,5); Serial.print(",");
    Serial.print(gz,5); Serial.print(",");
    Serial.print(tempC,2);
  }else{
    Serial.print("null,null,null,null,null,null,null");
  }

  Serial.print(",");
  Serial.print(leftTicks,2); Serial.print(",");
  Serial.print(rightTicks,2); Serial.print(",");

  for(int i=0;i<NUM_TOF;i++){

    if(dist[i]>=0) Serial.print(dist[i]);
    else Serial.print("null");

    if(i<NUM_TOF-1) Serial.print(",");
  }

  Serial.println();
}