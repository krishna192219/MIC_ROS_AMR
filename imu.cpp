#include "imu.h"
#include "Config.h"
#include "I2CBuses.h"


// ============================================================
// MPU9250 SETTINGS
// ============================================================

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

static bool imuInitialized = false;


// ============================================================
// WRITE REGISTER
// ============================================================

static void writeRegister(uint8_t reg, uint8_t value)
{
    I2C_Left.beginTransmission(MPU9250_ADDR);
    I2C_Left.write(reg);
    I2C_Left.write(value);
    I2C_Left.endTransmission();
}


// ============================================================
// READ REGISTERS
// ============================================================

static bool readRegisters(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length)
{
    I2C_Left.beginTransmission(MPU9250_ADDR);
    I2C_Left.write(reg);

    if (I2C_Left.endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t received =
        I2C_Left.requestFrom(
            MPU9250_ADDR,
            length
        );

    if (received != length)
    {
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = I2C_Left.read();
    }

    return true;
}


// ============================================================
// INITIALIZE MPU9250
// ============================================================

bool initIMU()
{
    // Wake sensor
    writeRegister(
        MPU_PWR_MGMT_1,
        0x00
    );

    delay(10);


    // Check WHO_AM_I
    uint8_t whoAmI = 0;

    if (!readRegisters(
            MPU_WHO_AM_I,
            &whoAmI,
            1))
    {
        Serial.println("ERROR: MPU9250 not responding.");
        return false;
    }


    Serial.print("MPU9250 WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);


    // Your working sensor has reported 0x70
    if (whoAmI != 0x68 &&
        whoAmI != 0x70 &&
        whoAmI != 0x71 &&
        whoAmI != 0x73)
    {
        Serial.println(
            "WARNING: Unexpected MPU9250 WHO_AM_I."
        );
    }


    // --------------------------------------------------------
    // Accelerometer
    // ±2g
    // --------------------------------------------------------

    writeRegister(
        0x1C,
        0x00
    );


    // --------------------------------------------------------
    // Gyroscope
    // ±250 deg/s
    // --------------------------------------------------------

    writeRegister(
        0x1B,
        0x00
    );


    imuInitialized = true;

    Serial.println("MPU9250 initialized.");

    return true;
}


// ============================================================
// UPDATE IMU
// ============================================================

void updateIMU()
{
    if (!imuInitialized)
    {
        return;
    }


    uint8_t buffer[14];


    if (!readRegisters(
            MPU_ACCEL_XOUT_H,
            buffer,
            14))
    {
        return;
    }


    // --------------------------------------------------------
    // RAW ACCELEROMETER
    // --------------------------------------------------------

    int16_t rawAx =
        ((int16_t)buffer[0] << 8) |
        buffer[1];

    int16_t rawAy =
        ((int16_t)buffer[2] << 8) |
        buffer[3];

    int16_t rawAz =
        ((int16_t)buffer[4] << 8) |
        buffer[5];


    // --------------------------------------------------------
    // RAW GYROSCOPE
    // --------------------------------------------------------

    int16_t rawGx =
        ((int16_t)buffer[8] << 8) |
        buffer[9];

    int16_t rawGy =
        ((int16_t)buffer[10] << 8) |
        buffer[11];

    int16_t rawGz =
        ((int16_t)buffer[12] << 8) |
        buffer[13];


    // --------------------------------------------------------
    // CONVERT ACCELEROMETER
    // --------------------------------------------------------

    float ax =
        rawAx / ACCEL_SCALE;

    float ay =
        rawAy / ACCEL_SCALE;

    float az =
        rawAz / ACCEL_SCALE;


    // --------------------------------------------------------
    // CONVERT GYROSCOPE
    // --------------------------------------------------------

    float gx =
        rawGx / GYRO_SCALE;

    float gy =
        rawGy / GYRO_SCALE;

    float gz =
        rawGz / GYRO_SCALE;


    // --------------------------------------------------------
    // APPLY YOUR EXISTING CALIBRATION
    // --------------------------------------------------------

    gx -= GYRO_BIAS_X;
    gy -= GYRO_BIAS_Y;
    gz -= GYRO_BIAS_Z;


    // --------------------------------------------------------
    // SERIAL OUTPUT
    // --------------------------------------------------------

    Serial.print("IMU,");

    Serial.print(ax, 3);
    Serial.print(",");

    Serial.print(ay, 3);
    Serial.print(",");

    Serial.print(az, 3);
    Serial.print(",");

    Serial.print(gx, 3);
    Serial.print(",");

    Serial.print(gy, 3);
    Serial.print(",");

    Serial.println(gz, 3);
}