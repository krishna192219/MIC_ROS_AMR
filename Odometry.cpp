#include "Odometry.h"
#include "Config.h"
#include "Encoders.h"
#include <math.h>


// ============================================================
// ROBOT POSE
// ============================================================

static float robotX = 0.0;
static float robotY = 0.0;
static float robotHeading = 0.0;


// ============================================================
// PREVIOUS ENCODER COUNTS
// ============================================================

static long previousLeftCount = 0;
static long previousRightCount = 0;


// ============================================================
// ODOMETRY UPDATE TIMING
// ============================================================

static unsigned long lastOdometryTime = 0;


// ============================================================
// INITIALIZE
// ============================================================

void initOdometry()
{
    robotX = 0.0;
    robotY = 0.0;
    robotHeading = 0.0;

    previousLeftCount = getTotalLeftCount();
    previousRightCount = getTotalRightCount();

    lastOdometryTime = millis();
}


// ============================================================
// UPDATE ODOMETRY
// ============================================================

void updateOdometry()
{
    long currentLeftCount = getTotalLeftCount();
    long currentRightCount = getTotalRightCount();


    long deltaLeftCount =
        currentLeftCount - previousLeftCount;

    long deltaRightCount =
        currentRightCount - previousRightCount;


    previousLeftCount = currentLeftCount;
    previousRightCount = currentRightCount;


    // --------------------------------------------------------
    // Convert encoder counts to wheel distance
    // --------------------------------------------------------

    float leftDistance =
        ((float)deltaLeftCount / COUNTS_PER_REV)
        * WHEEL_CIRCUMFERENCE;

    float rightDistance =
        ((float)deltaRightCount / COUNTS_PER_REV)
        * WHEEL_CIRCUMFERENCE;


    // --------------------------------------------------------
    // Differential-drive odometry
    // --------------------------------------------------------
    //
    // Your encoder convention is:
    //
    // LEFT  forward = positive
    // RIGHT forward = negative
    //
    // Therefore the physical right-wheel distance is
    // the negative of the stored right encoder distance.
    // --------------------------------------------------------

    rightDistance = -rightDistance;


    float centerDistance =
        (leftDistance + rightDistance) / 2.0;


    float deltaHeading =
        (rightDistance - leftDistance)
        / WHEEL_SEPARATION;


    // --------------------------------------------------------
    // Midpoint integration
    // --------------------------------------------------------

    float headingMid =
        robotHeading + (deltaHeading / 2.0);


    robotX += centerDistance * cos(headingMid);
    robotY += centerDistance * sin(headingMid);

    robotHeading += deltaHeading;


    // --------------------------------------------------------
    // Keep heading within -PI ... +PI
    // --------------------------------------------------------

    while (robotHeading > PI_VAL)
    {
        robotHeading -= 2.0 * PI_VAL;
    }

    while (robotHeading < -PI_VAL)
    {
        robotHeading += 2.0 * PI_VAL;
    }
}


// ============================================================
// GETTERS
// ============================================================

float getX()
{
    return robotX;
}


float getY()
{
    return robotY;
}


float getHeading()
{
    return robotHeading;
}


// ============================================================
// SERIAL OUTPUT
// ============================================================

void publishOdometry()
{
    Serial.print("POSE,");

    Serial.print(robotX, 4);
    Serial.print(",");

    Serial.print(robotY, 4);
    Serial.print(",");

    Serial.println(robotHeading, 4);
}