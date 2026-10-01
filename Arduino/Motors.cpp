#include "Motors.h"
#include "Config.h"


// ============================================================
// INITIALIZE MOTORS
// ============================================================

void initMotors()
{
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);


    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledcAttach(PWMA, 20000, 8);
    ledcAttach(PWMB, 20000, 8);


    stopMotors();
}


// ============================================================
// LEFT MOTOR FORWARD
// ============================================================

void leftForward(int pwm)
{
    pwm = constrain(pwm, 0, 255);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    ledcWrite(PWMA, pwm);
}


// ============================================================
// RIGHT MOTOR FORWARD
// ============================================================

void rightForward(int pwm)
{
    pwm = constrain(pwm, 0, 255);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    ledcWrite(PWMB, pwm);
}


// ============================================================
// LEFT MOTOR BACKWARD
// ============================================================

void leftBackward(int pwm)
{
    pwm = constrain(pwm, 0, 255);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    ledcWrite(PWMA, pwm);
}


// ============================================================
// RIGHT MOTOR BACKWARD
// ============================================================

void rightBackward(int pwm)
{
    pwm = constrain(pwm, 0, 255);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    ledcWrite(PWMB, pwm);
}


// ============================================================
// STOP LEFT
// ============================================================

void stopLeft()
{
    ledcWrite(PWMA, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
}


// ============================================================
// STOP RIGHT
// ============================================================

void stopRight()
{
    ledcWrite(PWMB, 0);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}


// ============================================================
// STOP BOTH
// ============================================================

void stopMotors()
{
    stopLeft();
    stopRight();
}