#include <Arduino.h>

#define PWM1 18
#define DIR1 19

#define PWM2 21
#define DIR2 22

namespace {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_RES_BITS = 8;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr size_t CMD_BUFFER_SIZE = 64;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void setupPwmPin(uint8_t pin) {
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS);
}

void writePwm(uint8_t pin, uint8_t value) {
  ledcWrite(pin, value);
}
#else
constexpr uint8_t LEFT_PWM_CH = 0;
constexpr uint8_t RIGHT_PWM_CH = 1;

void setupPwmPin(uint8_t pin) {
  if (pin == PWM1) {
    ledcSetup(LEFT_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PWM1, LEFT_PWM_CH);
    return;
  }

  ledcSetup(RIGHT_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PWM2, RIGHT_PWM_CH);
}

void writePwm(uint8_t pin, uint8_t value) {
  ledcWrite(pin == PWM1 ? LEFT_PWM_CH : RIGHT_PWM_CH, value);
}
#endif

void stopMotors() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  writePwm(PWM1, 0);
  writePwm(PWM2, 0);
}

void setMotor(uint8_t pwmPin, uint8_t dirPin, int pwmValue, bool forward) {
  pwmValue = constrain(pwmValue, 0, 255);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  writePwm(pwmPin, static_cast<uint8_t>(pwmValue));
}

bool parseCommand(const char *line, int &leftPwm, int &leftDir, int &rightPwm, int &rightDir) {
  return sscanf(line, "CMD,%d,%d,%d,%d", &leftPwm, &leftDir, &rightPwm, &rightDir) == 4;
}

char commandBuffer[CMD_BUFFER_SIZE];
size_t commandIndex = 0;
uint32_t lastCommandMs = 0;
}

void setup() {
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  setupPwmPin(PWM1);
  setupPwmPin(PWM2);
  stopMotors();

  Serial.begin(SERIAL_BAUD);
  lastCommandMs = millis();
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      commandBuffer[commandIndex] = '\0';

      int leftPwm = 0;
      int leftDir = 1;
      int rightPwm = 0;
      int rightDir = 1;
      if (parseCommand(commandBuffer, leftPwm, leftDir, rightPwm, rightDir)) {
        setMotor(PWM1, DIR1, leftPwm, leftDir != 0);
        setMotor(PWM2, DIR2, rightPwm, rightDir != 0);
        lastCommandMs = millis();
      }

      commandIndex = 0;
      continue;
    }

    if (commandIndex < CMD_BUFFER_SIZE - 1) {
      commandBuffer[commandIndex++] = ch;
    } else {
      commandIndex = 0;
    }
  }

  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopMotors();
    lastCommandMs = millis();
  }
}
