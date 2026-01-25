#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>

// ============================================================
// Servo Mechanical Configuration
// ------------------------------------------------------------
// This file defines the servo channel mapping, offset calibration,
// and motion limits for the robot's 8-DOF leg joints.
// ============================================================

#define NUM_SERVOS 8
#define NUM_ANGLES 40
#define HALF_FRAMES 20

// Servo channel mapping on PCA9685
extern int servoChannel[NUM_SERVOS];

// Calibration offsets to compensate mechanical installation errors
extern double servoOffset[NUM_SERVOS];

// Safe angle limits for each servo (min, max) in degrees
extern int servoAngleLimit[NUM_SERVOS][2];

// Gait motion profiles
extern double servoThighAngles_1[NUM_ANGLES/2];
extern double servoThighAngles_2[NUM_ANGLES/2];
extern double servoCalfAngles_1[NUM_ANGLES/2];
extern double servoCalfAngles_2[NUM_ANGLES/2];

#endif