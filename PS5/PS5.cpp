#include <Arduino.h>
#include <ps5Controller.h>

// ================= MOTOR PINS =================

#define PWM1 18
#define DIR1 19

#define PWM2 21
#define DIR2 22

// ================= VARIABLES =================

bool ps5Enabled = true;

int deadzone = 40;
int pwm = 0;

// ================= MOTOR CONTROL =================

void setMotor(int dirPin, int pwmPin, int speed)
{
    bool dir = speed >= 0;

    digitalWrite(dirPin, dir ? HIGH : LOW);
    analogWrite(pwmPin, abs(speed));
}

void stopMotors()
{
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    Serial.println("STOP");
}

// ================= JOYSTICK DRIVE =================

void joystickDrive()
{
    int throttle = ps5.LStickY();
    int turn = ps5.RStickX();

    if (abs(throttle) < deadzone)
        throttle = 0;
    if (abs(turn) < deadzone)
        turn = 0;

    if (throttle == 0 && turn == 0)
    {
        stopMotors();
        return;
    }

    int leftMotor = throttle + turn;
    int rightMotor = throttle - turn;

    leftMotor = constrain(leftMotor, -128, 127);
    rightMotor = constrain(rightMotor, -128, 127);

    int pwmLeft = map(abs(leftMotor), 0, 127, 0, 255);
    int pwmRight = map(abs(rightMotor), 0, 127, 0, 255);

    pwmLeft = constrain((leftMotor < 0 ? -pwmLeft : pwmLeft), -255, 255);
    pwmRight = constrain((rightMotor < 0 ? -pwmRight : pwmRight), -255, 255);

    setMotor(DIR1, PWM1, -pwmLeft);
    setMotor(DIR2, PWM2, -pwmRight);

    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print(" Turn: ");
    Serial.print(turn);

    Serial.print(" | L PWM: ");
    Serial.print(pwmLeft);

    Serial.print(" R PWM: ");
    Serial.println(pwmRight);
}

// ================= DPDT CONTROL =================

void dpdtControl()
{
    if (ps5.Up() && ps5.Triangle())
    {
        setMotor(DIR1, PWM1, -pwm);
        setMotor(DIR2, PWM2, -pwm);
        Serial.println("DPDT FORWARD");
    }

    else if (ps5.Down() && ps5.Cross())
    {
        setMotor(DIR1, PWM1, pwm);
        setMotor(DIR2, PWM2, pwm);
        Serial.println("DPDT BACKWARD");
    }

    else if (ps5.Up() && ps5.Cross())
    {
        setMotor(DIR1, PWM1, -pwm);
        setMotor(DIR2, PWM2, pwm);
        Serial.println("DPDT RIGHT");
    }

    else if (ps5.Down() && ps5.Triangle())
    {
        setMotor(DIR1, PWM1, pwm);
        setMotor(DIR2, PWM2, -pwm);
        Serial.println("DPDT LEFT");
    }

    else if (ps5.Up())
    {
        setMotor(DIR1, PWM1, -pwm);
        Serial.println("DPDT ONLY LEFTSIDE UP");
    }

    else if (ps5.Down())
    {
        setMotor(DIR1, PWM1, pwm);
        Serial.println("DPDT ONLY LEFTSIDE DOWN");
    }

    else if (ps5.Triangle())
    {
        setMotor(DIR2, PWM2, -pwm);
        Serial.println("DPDT ONLY RIGHTSIDE UP");
    }

    else if (ps5.Cross())
    {
        setMotor(DIR2, PWM2, pwm);
        Serial.println("DPDT ONLY RIGHTSIDE DOWN");
    }

    else
    {
        stopMotors();
    }
}

// ================= SETUP =================

void setup()
{
    Serial.begin(115200);

    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);

    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);

    ps5.begin("88:03:4c:fb:ca:67");

    Serial.println("PS5 Ready");
}

// ================= LOOP =================

void loop()
{

    if (!ps5.isConnected())
        return;

    // Toggle PS5 control
    if (ps5.event.button_down.square)
    {
        ps5Enabled = !ps5Enabled;
        stopMotors();

        Serial.println(ps5Enabled ? "PS5 ENABLED" : "PS5 DISABLED");
    }

    if (!ps5Enabled)
        return;

    // Read trigger for PWM
    pwm = ps5.L2Value();

    // Emergency stop
    if (ps5.Circle())
    {
        stopMotors();
        Serial.println("EMERGENCY STOP");
        return;
    }

    // DPDT has priority
    if (ps5.Up() || ps5.Down() || ps5.Triangle() || ps5.Cross())
    {
        dpdtControl();
    }
    else
    {
        joystickDrive();
    }
    // Battery check
    if (ps5.Left())
        Serial.println(ps5.Battery());
}