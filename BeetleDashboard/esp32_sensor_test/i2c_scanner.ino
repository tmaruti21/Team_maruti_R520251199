/*
 * I2C Scanner for ESP32
 * 
 * This sketch scans the I2C bus for connected devices.
 * It helps verify wiring and sensor addresses.
 * 
 * Pin Connections:
 *   SDA -> GPIO 21
 *   SCL -> GPIO 22
 */

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor
  
  Serial.println("\nI2C Scanner");
  Serial.print("SDA Pin: "); Serial.println(SDA_PIN);
  Serial.print("SCL Pin: "); Serial.println(SCL_PIN);
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("\nScanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
