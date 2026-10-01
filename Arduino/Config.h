#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// MOTOR DRIVER PINS
// ============================================================

// LEFT MOTOR
#define AIN2 27
#define AIN1 14
#define PWMA 12

// RIGHT MOTOR
#define BIN1 33
#define BIN2 25
#define PWMB 32

// STANDBY
#define STBY 26


// ============================================================
// ENCODER I2C PINS
// ============================================================

// Physical LEFT encoder
#define LEFT_SDA 16
#define LEFT_SCL 23

// Physical RIGHT encoder
#define RIGHT_SDA 21
#define RIGHT_SCL 22


// ============================================================
// AS5600
// ============================================================

#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_HIGH 0x0C


// ============================================================
// MPU9250
// ============================================================

#define MPU9250_ADDR 0x68

#define MPU_WHO_AM_I       0x75
#define MPU_PWR_MGMT_1     0x6B
#define MPU_ACCEL_XOUT_H   0x3B
#define MPU_GYRO_XOUT_H    0x43


// ============================================================
// ROBOT PARAMETERS
// ============================================================
const int COUNTS_PER_REV = 4096;

const float WHEEL_DIAMETER = 0.044;      // 44 mm
const float WHEEL_SEPARATION = 0.105;   // 105 mm
// 360 DEGREE SCAN
const float SCAN_SENSOR_OFFSET = 0.060;   // 6 cm

const long FULL_ROTATION_TARGET_COUNTS =
(long)((WHEEL_SEPARATION / WHEEL_DIAMETER) * COUNTS_PER_REV);


const float PI_VAL = 3.14159265359;

const float WHEEL_CIRCUMFERENCE =
    PI_VAL * WHEEL_DIAMETER;


// ============================================================
// SQUARE / MOTION PARAMETERS
// ============================================================

const float SIDE_LENGTH = 0.50;          // 50 cm
const float TURN_ANGLE = 90.0;

// Straight target counts
const long STRAIGHT_TARGET_COUNTS =
    (long)((SIDE_LENGTH / WHEEL_CIRCUMFERENCE)
           * COUNTS_PER_REV);

// Distance travelled by each wheel for a 90°
// in-place rotation
const float ROTATION_DISTANCE =
    (PI_VAL * WHEEL_SEPARATION) / 4.0;

// Rotation target counts
const long ROTATION_TARGET_COUNTS =
    (long)((ROTATION_DISTANCE / WHEEL_CIRCUMFERENCE)
           * COUNTS_PER_REV);


// ============================================================
// CLOSED LOOP PARAMETERS
// ============================================================

const float TARGET_RPM = 50.0;

// Feed-forward
const float FF_LEFT = 63.0;
const float FF_RIGHT = 63.0;


// ============================================================
// TURN CONTROL
// ============================================================

const float TURN_SYNC_KP = 0.06;

const int TURN_FAST_PWM = 40;
const int TURN_SLOW_PWM = 45;

const int TURN_MIN_PWM = 20;
const int TURN_MAX_PWM = 50;

const int TURN_SYNC_LIMIT = 10;


// ============================================================
// STRAIGHT CONTROL
// ============================================================

const float KP = 0.5;
const float KI = 0.0;

const float SYNC_KP = 0.005;

const int SYNC_TRIM_LIMIT = 15;


// ============================================================
// PWM LIMITS
// ============================================================

const int MIN_PWM = 0;
const int MAX_PWM = 90;

const int MAX_PWM_STEP = 5;


// ============================================================
// CONTROL LOOP
// ============================================================

const unsigned long CONTROL_PERIOD = 100;


// ============================================================
// MPU9250 CALIBRATION
// ============================================================

// Gyroscope bias obtained from your calibration

const float GYRO_BIAS_X = -2.889328;
const float GYRO_BIAS_Y = -1.042649;
const float GYRO_BIAS_Z = -0.273710;


#endif