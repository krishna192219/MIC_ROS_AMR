#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// Initialize MPU9250
bool initIMU();

// Read sensor and publish:
// IMU,ax,ay,az,gx,gy,gz
void updateIMU();

#endif  