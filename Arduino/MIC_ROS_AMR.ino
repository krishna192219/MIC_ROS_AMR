#include "Config.h"
#include "I2CBuses.h"
#include "Motors.h"
#include "Encoders.h"
#include "Odometry.h"
#include "imu.h"



#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>


// ============================================================
// VL53L0X
// ============================================================

Adafruit_VL53L0X lox;


// ============================================================
// SCAN SETTINGS
// ============================================================

// Publish one scan measurement every 50 ms
const unsigned long SCAN_INTERVAL = 50;

// Sensor is 6 cm in front of robot center
const float SENSOR_OFFSET_X = 0.060;


// ============================================================
// 360 DEGREE ROTATION
// ============================================================
//
// For differential drive:
//
// Distance travelled by each wheel for 360° robot rotation:
//
//     distance = PI * wheel separation
//
// Encoder counts:
//
//     counts = distance / wheel circumference
//            * counts per revolution
//
// ============================================================




// ============================================================
// VL53L0X INITIALIZATION
// ============================================================

bool initScanner()
{
    Serial.println("Initializing VL53L0X...");

    /*
       VL53L0X is connected to the RIGHT I2C bus:

       SDA = GPIO21
       SCL = GPIO22

       The RIGHT bus is the normal Wire bus.
    */

    if (!lox.begin(0x29, false, &Wire))
    {
        Serial.println("ERROR: VL53L0X not detected!");
        return false;
    }

    Serial.println("VL53L0X initialized.");

    /*
       IMPORTANT:

       We intentionally DO NOT call:

           startRangeContinuous()

       We use single-shot measurements through:

           rangingTest()

       This keeps the scanner simple and reliable while
       the robot is rotating.
    */

    Serial.println("VL53L0X single-shot ranging ready.");

    return true;
}


// ============================================================
// READ VL53L0X
// ============================================================

int readScannerDistance()
{
    VL53L0X_RangingMeasurementData_t measurement;

    lox.rangingTest(&measurement, false);

    Serial.print("VL53 STATUS = ");
    Serial.print(measurement.RangeStatus);

    Serial.print(" | DIST = ");
    Serial.println(measurement.RangeMilliMeter);

    if (measurement.RangeStatus != 0 && measurement.RangeStatus != 2)
    {
        return -1;
    }

    return measurement.RangeMilliMeter;
}


// ============================================================
// PUBLISH ONE SCAN MEASUREMENT
// ============================================================
//
// Format:
//
// SCAN,x,y,heading,distance_mm
//
// Example:
//
// SCAN,0.0000,0.0000,1.5708,436
//
// heading = robot odometry heading
//
// Because the VL53L0X is fixed facing forward,
// its world angle is approximately the robot heading.
//
// ============================================================

void publishScan()
{
    int distance = readScannerDistance();

    if (distance < 0)
    {
        Serial.println("SCAN_INVALID");
        return;
    }

    Serial.print("SCAN,");
    Serial.print(getX(), 4);
    Serial.print(",");
    Serial.print(getY(), 4);
    Serial.print(",");
    Serial.print(getHeading(), 4);
    Serial.print(",");
    Serial.println(distance);
}


// ============================================================
// 360 DEGREE ROTATION + SCANNING
// ============================================================

void rotate360AndScan()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("STARTING 360 DEG SCAN");
    Serial.println("================================");

    Serial.print("Target rotation counts: ");
    Serial.println(FULL_ROTATION_TARGET_COUNTS);

    Serial.println("SCAN_START");


    // --------------------------------------------------------
    // Save starting encoder totals
    // --------------------------------------------------------

    long startLeft = getTotalLeftCount();
    long startRight = getTotalRightCount();


    // --------------------------------------------------------
    // Local rotation counters
    // --------------------------------------------------------

    long leftMoved = 0;
    long rightMoved = 0;


    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    unsigned long startTime = millis();
    unsigned long lastControl = millis();
    unsigned long lastScan = millis();


    // --------------------------------------------------------
    // Initial motor speeds
    // --------------------------------------------------------

    int leftPWM = TURN_FAST_PWM;
    int rightPWM = TURN_FAST_PWM;


    // --------------------------------------------------------
    // Start rotation
    //
    // LEFT  = forward
    // RIGHT = backward
    //
    // This is the direction already tested successfully.
    // --------------------------------------------------------

    leftForward(leftPWM);
    rightBackward(rightPWM);


    // ========================================================
    // ROTATION LOOP
    // ========================================================

    while (true)
    {
        // ----------------------------------------------------
        // Update encoders
        // ----------------------------------------------------

        updateEncoders();


        // ----------------------------------------------------
        // Update odometry
        // ----------------------------------------------------

        updateOdometry();


        // ----------------------------------------------------
        // Calculate local movement
        // ----------------------------------------------------

        leftMoved =
            labs(getTotalLeftCount() - startLeft);

        rightMoved =
            labs(getTotalRightCount() - startRight);


        // ----------------------------------------------------
        // Scan every 50 ms
        // ----------------------------------------------------

        unsigned long now = millis();

        if (now - lastScan >= SCAN_INTERVAL)
        {
        

            lastScan = now;

            publishOdometry();
            publishScan();
        }


        // ----------------------------------------------------
        // Synchronize left/right rotation
        // ----------------------------------------------------

        if (now - lastControl >= 50)
        {
            long turnError =
                leftMoved - rightMoved;


            float syncTrim =
                TURN_SYNC_KP * (float)turnError;


            syncTrim = constrain(
                syncTrim,
                -TURN_SYNC_LIMIT,
                TURN_SYNC_LIMIT
            );


            int desiredLeftPWM =
                TURN_FAST_PWM - (int)syncTrim;


            int desiredRightPWM =
                TURN_FAST_PWM + (int)syncTrim;


            desiredLeftPWM = constrain(
                desiredLeftPWM,
                TURN_MIN_PWM,
                TURN_MAX_PWM
            );


            desiredRightPWM = constrain(
                desiredRightPWM,
                TURN_MIN_PWM,
                TURN_MAX_PWM
            );


            leftForward(desiredLeftPWM);
            rightBackward(desiredRightPWM);


            lastControl = now;
        }


        // ----------------------------------------------------
        // Check if full 360° rotation is complete
        // ----------------------------------------------------

        if (
            leftMoved >= FULL_ROTATION_TARGET_COUNTS &&
            rightMoved >= FULL_ROTATION_TARGET_COUNTS
        )
        {
            break;
        }


        // ----------------------------------------------------
        // Safety timeout
        // ----------------------------------------------------

        if (millis() - startTime > 15000)
        {
            Serial.println("SCAN ROTATION TIMEOUT!");
            break;
        }


        // ----------------------------------------------------
        // Small delay
        // ----------------------------------------------------

        delay(2);
    }


    // ========================================================
    // STOP ROBOT
    // ========================================================

    stopMotors();


    // --------------------------------------------------------
    // Final encoder + odometry update
    // --------------------------------------------------------

    updateEncoders();
    updateOdometry();


    // --------------------------------------------------------
    // One final scan
    // --------------------------------------------------------

    publishScan();


    // --------------------------------------------------------
    // Scan finished
    // --------------------------------------------------------

    Serial.println("SCAN_END");

    Serial.println("================================");
    Serial.println("360 DEG SCAN COMPLETE");
    Serial.println("================================");


    Serial.print("Left rotation counts: ");
    Serial.println(leftMoved);


    Serial.print("Right rotation counts: ");
    Serial.println(rightMoved);


    Serial.print("Final heading: ");
    Serial.println(getHeading(), 4);


    Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println("=================================");
    Serial.println("       AMR CONTROLLER");
    Serial.println("=================================");


    // ========================================================
    // I2C
    // ========================================================

    initI2CBuses();

    Serial.println("I2C buses initialized.");


    // ========================================================
    // MOTORS
    // ========================================================

    initMotors();

    Serial.println("Motors initialized.");


    // ========================================================
    // ENCODERS
    // ========================================================

    if (!initEncoders())
    {
        Serial.println(
            "ERROR: Encoder initialization failed!"
        );
    }
    else
    {
        Serial.println("Encoders initialized.");
    }


    // ========================================================
    // ODOMETRY
    // ========================================================

    initOdometry();

    Serial.println("Odometry initialized.");


    // ========================================================
    // IMU
    // ========================================================

    if (!initIMU())
    {
        Serial.println(
            "ERROR: IMU initialization failed!"
        );
    }
    else
    {
        Serial.println("IMU initialized.");
    }


    // ========================================================
    // VL53L0X
    // ========================================================

    if (!initScanner())
    {
        Serial.println(
            "ERROR: Scanner initialization failed!"
        );
    }
    else
    {
        Serial.println("Scanner initialized.");
    }


    // ========================================================
    // SYSTEM READY
    // ========================================================

    Serial.println("=================================");
    Serial.println("       SYSTEM READY");
    Serial.println("=================================");

    Serial.println();

    Serial.println("Active sensors:");
    Serial.println("  - Left AS5600 encoder");
    Serial.println("  - Right AS5600 encoder");
    Serial.println("  - MPU9250 IMU");
    Serial.println("  - VL53L0X distance sensor");

    Serial.println();

    Serial.println("Outputs:");
    Serial.println("  ENC,left_count,right_count");
    Serial.println("  POSE,x,y,heading");
    Serial.println("  IMU,ax,ay,az,gx,gy,gz");
    Serial.println("  SCAN,x,y,heading,distance");

    Serial.println();

    Serial.print("Full rotation target counts: ");
    Serial.println(FULL_ROTATION_TARGET_COUNTS);

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    static bool scanDone = false;
    static unsigned long lastPoseTime = 0;

    if (!scanDone)
    {
        delay(2000);

        rotate360AndScan();

        scanDone = true;
    }

    updateEncoders();
    updateOdometry();

    // Publish odometry at 20 Hz
    if (millis() - lastPoseTime >= 50)
    {
        lastPoseTime = millis();
        publishOdometry();
    }

    delay(10);
}