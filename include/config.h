#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// ROBOTIC ARM HARDWARE & KINEMATICS CONFIGURATION
// Semester 3 Robotics Project
// ============================================================================

// ----------------------------------------------------------------------------
// 1. PIN ASSIGNMENTS (PWM Pins on Arduino Uno)
// ----------------------------------------------------------------------------
#define PIN_BASE_SERVO      9   // Base yaw rotation (rotates entire arm structure)
#define PIN_SHOULDER_SERVO  3   // Shoulder pitch joint
#define PIN_ELBOW_SERVO     11  // Elbow pitch joint
#define PIN_CLAMP_SERVO     5   // Wrist / End-Effector clamp (gripper)

// ----------------------------------------------------------------------------
// 2. PHYSICAL LINK DIMENSIONS (in centimeters)
// Measured along kinematic axes of rotation
// ----------------------------------------------------------------------------
#define L1_BASE_HEIGHT      4.5f   // Height from ground / base mount to shoulder pivot
#define L2_UPPER_ARM        12.0f  // Distance from shoulder pivot to elbow pivot
#define L3_FOREARM_CLAMP    9.0f   // Distance from elbow pivot to clamp end-effector tip

// ----------------------------------------------------------------------------
// 3. JOINT SERVO LIMITS & CALIBRATION (in degrees [0° - 180°])
// Safety bounds to prevent physical collisions and mechanical strain on SG90s
// ----------------------------------------------------------------------------
#define BASE_MIN_DEG        0
#define BASE_MAX_DEG        180
#define BASE_HOME_DEG       90

#define SHOULDER_MIN_DEG    15
#define SHOULDER_MAX_DEG    165
#define SHOULDER_HOME_DEG   90

#define ELBOW_MIN_DEG       15
#define ELBOW_MAX_DEG       165
#define ELBOW_HOME_DEG      90

// Clamp / Gripper calibration angles
#define CLAMP_CLOSED_DEG    30     // Angle when clamp grips an object
#define CLAMP_OPEN_DEG      100    // Angle when clamp is fully opened
#define CLAMP_MIN_DEG       20
#define CLAMP_MAX_DEG       120

// ----------------------------------------------------------------------------
// 4. MOTION INTERPOLATION & COMMUNICATION SETTINGS
// ----------------------------------------------------------------------------
#define SERVO_STEP_DELAY_MS 15     // Milliseconds per 1° interpolation step (smooth motion)
#define SERIAL_BAUD_RATE    115200 // Baud rate for Serial Monitor

#endif // CONFIG_H
