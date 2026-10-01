#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

// Initialize odometry state
void initOdometry();

// Update pose from encoder counts
void updateOdometry();

// Get current robot pose
float getX();
float getY();
float getHeading();

// Print current pose
void publishOdometry();

#endif  