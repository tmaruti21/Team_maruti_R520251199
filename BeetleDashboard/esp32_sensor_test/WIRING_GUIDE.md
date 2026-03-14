# BEETLE ESP32 WROOM - Wiring Guide

## Pin Configuration Summary

### I2C Bus (Shared by MPU6050 and all ToF sensors)
```
ESP32           Sensors
-----           -------
GPIO21 (SDA) -> SDA (all I2C devices)
GPIO22 (SCL) -> SCL (all I2C devices)
3.3V         -> VCC (all sensors)
GND          -> GND (all sensors)
```

---

## MPU6050 Wiring

| MPU6050 Pin | ESP32 Pin |
|-------------|-----------|
| VCC         | 3.3V      |
| GND         | GND       |
| SCL         | GPIO22    |
| SDA         | GPIO21    |
| XDA         | NC        |
| XCL         | NC        |
| AD0         | GND       |
| INT         | NC (optional: GPIO interrupt) |

---

## Encoder Wiring (6 Encoders)

Each encoder requires 2 GPIO pins (A and B channels) for quadrature decoding.

| Encoder | Channel A | Channel B | Notes |
|---------|-----------|-----------|-------|
| ENC1    | GPIO32    | GPIO33    | Left Front |
| ENC2    | GPIO25    | GPIO26    | Left Middle |
| ENC3    | GPIO27    | GPIO14    | Left Rear |
| ENC4    | GPIO12    | GPIO13    | Right Front |
| ENC5    | GPIO15    | GPIO2     | Right Middle |
| ENC6    | GPIO4     | GPIO16    | Right Rear |

**Encoder Power:**
- VCC -> 5V (most encoders need 5V)
- GND -> GND
- Use voltage divider (5V to 3.3V) if encoder outputs 5V logic

**Important Notes:**
- GPIO12 must be LOW at boot (may need modification)
- GPIO2 has onboard LED, might flash during counting
- GPIO15 must be LOW at boot for normal boot

---

## VL53L0X ToF Sensors Wiring (7 Sensors)

All ToF sensors share the I2C bus. Each needs an XSHUT pin for address assignment.

| Sensor | I2C Address | XSHUT Pin | Position |
|--------|-------------|-----------|----------|
| ToF1   | 0x30        | GPIO17    | Front Left |
| ToF2   | 0x31        | GPIO5     | Front Center Left |
| ToF3   | 0x32        | GPIO18    | Front Center |
| ToF4   | 0x33        | GPIO19    | Front Center Right |
| ToF5   | 0x34        | GPIO23    | Front Right |
| ToF6   | 0x35        | GPIO34*   | Left Side |
| ToF7   | 0x36        | GPIO35*   | Right Side |

**⚠️ GPIO34 and GPIO35 are INPUT ONLY on ESP32!**
Alternative pins for ToF6/ToF7:
- Use a GPIO expander (PCF8574 or PCA9685)
- Or use 74HC595 shift register
- Or change to available GPIO pins

### Alternative XSHUT Pins:
| Sensor | Alternative Pin |
|--------|-----------------|
| ToF6   | GPIO0 (careful - boot pin) |
| ToF7   | GPIO1 (TX - if not using) |

---

## ESP32 WROOM Pin Map

```
                    ┌─────────────────────┐
                    │      ESP32 WROOM    │
                    │                     │
              3V3 ──┤ 3V3           GND ├── GND
               EN ──┤ EN            D23 ├── GPIO23 (ToF5 XSHUT)
        (ADC)  VP ──┤ VP            D22 ├── GPIO22 (I2C SCL)
        (ADC)  VN ──┤ VN            TX0 ├── GPIO1 (TX)
(ToF6) GPIO34 ──┤ D34           RX0 ├── GPIO0 (RX)
(ToF7) GPIO35 ──┤ D35           D21 ├── GPIO21 (I2C SDA)
(ENC1A) GPIO32 ──┤ D32           D19 ├── GPIO19 (ToF4 XSHUT)
(ENC1B) GPIO33 ──┤ D33           D18 ├── GPIO18 (ToF3 XSHUT)
(ENC2A) GPIO25 ──┤ D25            D5 ├── GPIO5  (ToF2 XSHUT)
(ENC2B) GPIO26 ──┤ D26           D17 ├── GPIO17 (ToF1 XSHUT)
(ENC3A) GPIO27 ──┤ D27           D16 ├── GPIO16 (ENC6B)
(ENC3B) GPIO14 ──┤ D14            D4 ├── GPIO4  (ENC6A)
(ENC4A) GPIO12 ──┤ D12            D2 ├── GPIO2  (ENC5B)
(ENC4B) GPIO13 ──┤ D13           D15 ├── GPIO15 (ENC5A)
              GND ──┤ GND          GND ├── GND
              VIN ──┤ VIN          3V3 ├── 3V3
                    │                     │
                    │      [USB PORT]     │
                    └─────────────────────┘
```

---

## Wiring Checklist

### Before Powering On:
- [ ] All I2C devices share SDA (GPIO21) and SCL (GPIO22)
- [ ] All GND connections are common
- [ ] MPU6050 gets 3.3V power
- [ ] VL53L0X sensors get 3.3V power
- [ ] Encoders get appropriate power (3.3V or 5V with level shifter)
- [ ] ToF XSHUT pins are connected correctly
- [ ] No short circuits between VCC and GND

### After Power On:
1. Open Serial Monitor at 115200 baud
2. Run simple_test.ino first
3. Check I2C scanner finds devices:
   - 0x68 = MPU6050
   - 0x29 = VL53L0X (default, before address change)
4. Test each sensor individually
5. Then run full esp32_sensor_test.ino

---

## Libraries to Install (Arduino IDE)

1. **Adafruit MPU6050** (and dependencies)
   - Search: "Adafruit MPU6050"
   - Install with dependencies

2. **Adafruit VL53L0X**
   - Search: "Adafruit VL53L0X"
   - Install with dependencies

3. **ESP32 Board Package**
   - Board Manager URL: https://dl.espressif.com/dl/package_esp32_index.json
   - Install "ESP32 by Espressif Systems"

---

## Troubleshooting

### MPU6050 Not Found:
- Check AD0 pin (GND = 0x68, VCC = 0x69)
- Verify 3.3V power
- Check I2C pull-up resistors (4.7kΩ usually built-in)

### ToF Not Working:
- Make sure XSHUT pins are set HIGH to enable sensor
- Default address 0x29 can have collisions
- Try one sensor at a time

### Encoder Not Counting:
- Check encoder VCC (some need 5V)
- Verify A/B channels are correct
- Use oscilloscope to check signal quality

### WiFi Connection Failed:
- Verify SSID and password
- ESP32 only supports 2.4GHz WiFi
- Check server IP is reachable
