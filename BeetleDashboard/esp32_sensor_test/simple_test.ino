/*
 * ============================================
 * BEETLE ROBOT - SIMPLE SENSOR TEST
 * ============================================
 * 
 * This is a simplified test sketch to verify
 * each sensor individually before full integration.
 * 
 * Uncomment the section you want to test.
 */

#include <Wire.h>

// ============================================
// I2C PINS
// ============================================
#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n====================================");
  Serial.println("  BEETLE ESP32 - SIMPLE SENSOR TEST");
  Serial.println("====================================\n");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Run I2C Scanner to find connected devices
  scanI2C();
}

void loop() {
  // Uncomment ONE of these to test
  
  testMPU6050();
  // testSingleEncoder();
  // testSingleToF();
  
  delay(500);
}

// ============================================
// I2C SCANNER
// ============================================
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("  Found device at 0x%02X", address);
      
      // Identify common devices
      if (address == 0x68) Serial.print(" (MPU6050)");
      if (address == 0x29) Serial.print(" (VL53L0X default)");
      if (address >= 0x30 && address <= 0x36) Serial.print(" (VL53L0X custom)");
      
      Serial.println();
      deviceCount++;
    }
  }
  
  Serial.printf("\nFound %d I2C devices\n\n", deviceCount);
}

// ============================================
// MPU6050 TEST (Raw registers - no library)
// ============================================
void testMPU6050() {
  const uint8_t MPU_ADDR = 0x68;
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);
  
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temp
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  int16_t rawGy = Wire.read() << 8 | Wire.read();
  int16_t rawGz = Wire.read() << 8 | Wire.read();
  
  // Convert to real units (default ranges: ±2g, ±250°/s)
  float ax = rawAx / 16384.0 * 9.81;  // m/s²
  float ay = rawAy / 16384.0 * 9.81;
  float az = rawAz / 16384.0 * 9.81;
  float gx = rawGx / 131.0;  // °/s
  float gy = rawGy / 131.0;
  float gz = rawGz / 131.0;
  
  Serial.println("--- MPU6050 Test ---");
  Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n", ax, ay, az);
  Serial.printf("Gyro:  X=%.2f Y=%.2f Z=%.2f °/s\n", gx, gy, gz);
  Serial.println();
}

// ============================================
// SINGLE ENCODER TEST
// ============================================
#define TEST_ENC_A 32
#define TEST_ENC_B 33

volatile long testEncoderCount = 0;

void IRAM_ATTR testEncoderISR() {
  if (digitalRead(TEST_ENC_A) == digitalRead(TEST_ENC_B)) {
    testEncoderCount++;
  } else {
    testEncoderCount--;
  }
}

void setupSingleEncoder() {
  pinMode(TEST_ENC_A, INPUT_PULLUP);
  pinMode(TEST_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TEST_ENC_A), testEncoderISR, CHANGE);
}

void testSingleEncoder() {
  static bool initialized = false;
  if (!initialized) {
    setupSingleEncoder();
    initialized = true;
    Serial.println("Encoder initialized on pins 32/33");
  }
  
  Serial.printf("Encoder Count: %ld\n", testEncoderCount);
}

// ============================================
// SINGLE VL53L0X ToF TEST
// ============================================
#define TEST_TOF_XSHUT 17

void testSingleToF() {
  const uint8_t TOF_ADDR = 0x29;  // Default address
  
  // Check if ToF is present
  Wire.beginTransmission(TOF_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println("ToF sensor not found at 0x29!");
    Serial.println("Make sure XSHUT is HIGH (or not connected)");
    return;
  }
  
  // Read distance (simplified - for full accuracy use library)
  // This is a basic check to see if sensor responds
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(0xC0);  // Device ID register
  Wire.endTransmission();
  Wire.requestFrom(TOF_ADDR, 1);
  
  if (Wire.available()) {
    byte deviceId = Wire.read();
    Serial.printf("ToF Device ID: 0x%02X (should be 0xEE for VL53L0X)\n", deviceId);
  }
  
  Serial.println("For full distance readings, use Adafruit_VL53L0X library");
}
