#include "Encoders.h"
#include "Config.h"
#include "I2CBuses.h"


// ============================================================
// ENCODER STATE
// ============================================================

// ------------------------------------------------------------
// Segment counts
// ------------------------------------------------------------
// These are reset whenever a new movement starts.
// ------------------------------------------------------------

static long leftCount = 0;
static long rightCount = 0;


// ------------------------------------------------------------
// Cumulative counts
// ------------------------------------------------------------
// These NEVER reset.
// ROS uses these for odometry.
// ------------------------------------------------------------

static long totalLeftCount = 0;
static long totalRightCount = 0;


// ------------------------------------------------------------
// Previous raw AS5600 values
// ------------------------------------------------------------

static int lastLeftRaw = 0;
static int lastRightRaw = 0;


// ------------------------------------------------------------
// Initialization state
// ------------------------------------------------------------

static bool initialized = false;


// ------------------------------------------------------------
// Serial publishing
// ------------------------------------------------------------

static unsigned long lastPublishTime = 0;

const unsigned long ENCODER_PUBLISH_PERIOD = 50;


// ============================================================
// RAW AS5600 READ
// ============================================================

static int readAngle(TwoWire &bus)
{
    bus.beginTransmission(AS5600_ADDR);

    bus.write(AS5600_RAW_ANGLE_HIGH);

    if (bus.endTransmission(false) != 0)
    {
        return -1;
    }

    if (bus.requestFrom(AS5600_ADDR, 2) != 2)
    {
        return -1;
    }

    int highByte = bus.read();
    int lowByte = bus.read();

    int angle =
        ((highByte << 8) | lowByte) & 0x0FFF;

    return angle;
}


// ============================================================
// RELIABLE AS5600 READ
// ============================================================

static int readAngleReliable(TwoWire &bus)
{
    for (int i = 0; i < 3; i++)
    {
        int angle = readAngle(bus);

        if (angle >= 0)
        {
            return angle;
        }

        delayMicroseconds(100);
    }

    return -1;
}


// ============================================================
// AS5600 WRAP-AROUND
// ============================================================

static int angleDifference(int current, int previous)
{
    int difference = current - previous;

    if (difference > 2048)
    {
        difference -= 4096;
    }

    if (difference < -2048)
    {
        difference += 4096;
    }

    return difference;
}


// ============================================================
// INITIALIZE ENCODERS
// ============================================================

bool initEncoders()
{
    // --------------------------------------------------------
    // Read initial raw angles
    // --------------------------------------------------------

    int leftRaw = readAngleReliable(I2C_Left);
    int rightRaw = readAngleReliable(Wire);

    if (leftRaw < 0 || rightRaw < 0)
    {
        Serial.println("ERROR: AS5600 initialization failed.");

        if (leftRaw < 0)
        {
            Serial.println("ERROR: LEFT AS5600 read failed.");
        }

        if (rightRaw < 0)
        {
            Serial.println("ERROR: RIGHT AS5600 read failed.");
        }

        return false;
    }


    lastLeftRaw = leftRaw;
    lastRightRaw = rightRaw;


    // --------------------------------------------------------
    // Reset counters
    // --------------------------------------------------------

    leftCount = 0;
    rightCount = 0;

    totalLeftCount = 0;
    totalRightCount = 0;


    initialized = true;


    Serial.println("AS5600 encoders initialized.");

    Serial.print("Initial LEFT angle: ");
    Serial.println(lastLeftRaw);

    Serial.print("Initial RIGHT angle: ");
    Serial.println(lastRightRaw);


    return true;
}


// ============================================================
// UPDATE LEFT ENCODER
// ============================================================

static void updateLeftEncoder()
{
    int currentRaw = readAngleReliable(I2C_Left);

    if (currentRaw < 0)
    {
        return;
    }

    int delta = angleDifference(
        currentRaw,
        lastLeftRaw
    );

    // --------------------------------------------------------
    // LEFT ENCODER SIGN
    // --------------------------------------------------------
    //
    // Physical forward motion is positive on the LEFT side.
    //
    // This matches the working controller convention.
    // --------------------------------------------------------

    delta = delta;

    leftCount += delta;
    totalLeftCount += delta;

    lastLeftRaw = currentRaw;
}


// ============================================================
// UPDATE RIGHT ENCODER
// ============================================================

static void updateRightEncoder()
{
    int currentRaw = readAngleReliable(Wire);

    if (currentRaw < 0)
    {
        return;
    }

    int delta = angleDifference(
        currentRaw,
        lastRightRaw
    );

    // --------------------------------------------------------
    // RIGHT ENCODER SIGN
    // --------------------------------------------------------
    //
    // For physical forward motion the right AS5600 direction
    // is opposite to the left encoder.
    //
    // Therefore the sign is inverted here.
    //
    // This gives:
    //
    // physical forward:
    // LEFT  = positive
    // RIGHT = negative
    //
    // which is the convention used by the working firmware.
    // --------------------------------------------------------

    delta = delta;

    rightCount += delta;
    totalRightCount += delta;

    lastRightRaw = currentRaw;
}


// ============================================================
// UPDATE BOTH ENCODERS
// ============================================================

void updateEncoders()
{
    if (!initialized)
    {
        return;
    }

    updateLeftEncoder();
    updateRightEncoder();
}


// ============================================================
// RESET SEGMENT COUNTS
// ============================================================

void resetSegmentCounts()
{
    leftCount = 0;
    rightCount = 0;
}


// ============================================================
// GET SEGMENT COUNTS
// ============================================================

long getLeftCount()
{
    return leftCount;
}


long getRightCount()
{
    return rightCount;
}


// ============================================================
// GET CUMULATIVE COUNTS
// ============================================================

long getTotalLeftCount()
{
    return totalLeftCount;
}


long getTotalRightCount()
{
    return totalRightCount;
}


// ============================================================
// PUBLISH ENCODER DATA
// ============================================================

void publishEncoderData()
{
    unsigned long now = millis();

    if (now - lastPublishTime < ENCODER_PUBLISH_PERIOD)
    {
        return;
    }

    lastPublishTime = now;


    Serial.print("ENC,");
    Serial.print(totalLeftCount);
    Serial.print(",");
    Serial.println(totalRightCount);
}