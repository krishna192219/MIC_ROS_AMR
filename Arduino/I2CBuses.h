#ifndef I2C_BUSES_H
#define I2C_BUSES_H

#include <Wire.h>

// ============================================================
// SHARED I2C BUSES
// ============================================================

// Bus 0
// GPIO21 SDA
// GPIO22 SCL
//
// Used by:
// - Right AS5600
// - VL53L0X
extern TwoWire I2C_Right;


// Bus 1
// GPIO16 SDA
// GPIO23 SCL
//
// Used by:
// - Left AS5600
// - MPU9250
extern TwoWire I2C_Left;


// ============================================================
// INITIALIZATION
// ============================================================

void initI2CBuses();

#endif