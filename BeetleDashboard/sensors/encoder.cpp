#include <Arduino.h>

#define ENCODER_LEFT_A 26
#define ENCODER_LEFT_B 27
#define ENCODER_RIGHT_A 25
#define ENCODER_RIGHT_B 33

#define WHEEL_DIAMETER 0.075
#define WHEEL_BASE 0.26
#define PULSES_PER_REV 138

volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;
volatile int leftDirection = 1;
volatile int rightDirection = 1;

float linearVelocityX = 0.0;
float angularVelocity = 0.0;
float distanceLeft = 0.0;
float distanceRight = 0.0;
float distanceCoveredX = 0.0;

unsigned long lastUpdateTime = 0;

// LEFT ENCODER ISR
void IRAM_ATTR handleLeftEncoder() {
  int channelA = digitalRead(ENCODER_LEFT_A);
  int channelB = digitalRead(ENCODER_LEFT_B);

  if (channelA == channelB) leftDirection = 1;
  else leftDirection = -1;

  leftPulseCount += leftDirection;
}

// RIGHT ENCODER ISR
void IRAM_ATTR handleRightEncoder() {
  int channelA = digitalRead(ENCODER_RIGHT_A);
  int channelB = digitalRead(ENCODER_RIGHT_B);

  if (channelA == channelB) rightDirection = 1;
  else rightDirection = -1;

  rightPulseCount += rightDirection;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoder, RISING);

  lastUpdateTime = millis();

  // CSV HEADER
  Serial.println("t_ms,left_pulses,right_pulses,left_rpm,right_rpm,linear_vel_mps,angular_vel_rads,dist_left_m,dist_right_m,dist_x_m");
}

void loop() {

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastUpdateTime;

  if (elapsedTime >= 100) {   // 10 Hz update (better than 1 sec)

    float wheelCircumference = WHEEL_DIAMETER * PI;

    float leftRevolutions = leftPulseCount / (float)PULSES_PER_REV;
    float rightRevolutions = rightPulseCount / (float)PULSES_PER_REV;

    float leftRPM = leftRevolutions * (60000.0 / elapsedTime);
    float rightRPM = rightRevolutions * (60000.0 / elapsedTime);

    float leftAngularVelocity = leftRPM * (2 * PI / 60);
    float rightAngularVelocity = rightRPM * (2 * PI / 60);

    float leftLinearVelocity = leftAngularVelocity * (WHEEL_DIAMETER / 2.0);
    float rightLinearVelocity = rightAngularVelocity * (WHEEL_DIAMETER / 2.0);

    linearVelocityX = (leftLinearVelocity + rightLinearVelocity) / 2.0;

    angularVelocity = (rightLinearVelocity - leftLinearVelocity) / WHEEL_BASE;

    distanceLeft += leftRevolutions * wheelCircumference * leftDirection;
    distanceRight += rightRevolutions * wheelCircumference * rightDirection;

    distanceCoveredX = (distanceLeft + distanceRight) / 2.0;

    // CSV OUTPUT
    Serial.print(currentTime); Serial.print(",");
    Serial.print(leftPulseCount); Serial.print(",");
    Serial.print(rightPulseCount); Serial.print(",");
    Serial.print(leftRPM,3); Serial.print(",");
    Serial.print(rightRPM,3); Serial.print(",");
    Serial.print(linearVelocityX,4); Serial.print(",");
    Serial.print(angularVelocity,4); Serial.print(",");
    Serial.print(distanceLeft,4); Serial.print(",");
    Serial.print(distanceRight,4); Serial.print(",");
    Serial.println(distanceCoveredX,4);

    leftPulseCount = 0;
    rightPulseCount = 0;

    lastUpdateTime = currentTime;
  }
}