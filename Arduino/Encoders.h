#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// ============================================================
// INITIALIZATION
// ============================================================

bool initEncoders();


// ============================================================
// UPDATE
// ============================================================

// Read both AS5600 encoders and update all counts.
void updateEncoders();


// ============================================================
// SEGMENT COUNTS
// ============================================================

// Counts for the current movement.
// These can be reset without affecting cumulative odometry.

long getLeftCount();
long getRightCount();


// ============================================================
// CUMULATIVE COUNTS
// ============================================================

// These NEVER reset during normal operation.
// ROS odometry will use these.

long getTotalLeftCount();
long getTotalRightCount();


// ============================================================
// RESET MOVEMENT COUNTS
// ============================================================

// Resets only the segment counts.
// Cumulative counts remain unchanged.

void resetSegmentCounts();


// ============================================================
// SERIAL OUTPUT
// ============================================================

// Publishes:
//
// ENC,leftCount,rightCount
//
// at the configured interval.

void publishEncoderData();

#endif