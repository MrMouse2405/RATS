#pragma once

#include "Pololu3piPlus32U4.h"
#include "Option.h"

// Time (ms) spent sampling by IR Sensors
#define IRSENSOR_SAMPLING_TIME 2000

// time (ms) to wait until a decision is made for Dot Signs
#define DOT_SIGN_TIMEOUT 1

// IR Sensor result values below this will be ignored
#define NOISE_THRESHOLD 50

// IR Sensor result values below this will be considered to 
// be not on black line.
#define LINE_THRESHOLD 500

// Maximum & Minimum speed the motors will be allowed to turn.
#define MAX_SPEED 200 // 1.5 m/s
#define MIN_SPEED 0

#define SLOW_MAX_SPEED 50
#define SLOW_MIN_SPEED 0

// Speed of motors while calibration
#define CALIBRATION_SPEED 50 // very slow

/*
 * PID Constants
 *
 * This configuration uses a default proportional constant of 1/4
 * and a derivative constant of 1, which appears to perform well at low speeds.
 * Note: Adapted from Pololu3piplus documentation.
 */
#define PROPORTIONAL_CONSTANT 64 // coefficient of the P term * 256
#define DERIVATIVE_CONSTANT 256  // coefficient of the D term * 256

/**
 * 
 * Magnetic Anamoly Threshold
 * 
 */
#define MAG_THRESHOLD 1000 //TODO: Measure and adjust

#define MAG_DEBOUNCE_THRESHOLD 1000

/**
 * 
 * Robot Odometry Constants
 * 
 */
#define WHEEL_DISTANCE 96.0   // mm
#define TICKS_PER_REV  12.0      // Adjust based on your encoder
#define WHEEL_DIAMETER 32.0     // mm

/**
 * 
 * Frame Rate Constants
 * 
 */

#define MILLISECONDS_PER_FRAME 10

/*
 *
 * Buzzer Notes
 *
 */

#define GO_SEQUENCE "L16 cdegreg4"
#define BEEP_SEQUENCE ">g32>>c32"

/**
 * 
 *  Game Loop Related Types
 * 
 */
typedef unsigned long milliseconds;

/**
 * 
 *  Sensor Related Types
 * 
 */
typedef Option<int> LineDetectionResult;

template <typename T>
struct Vec2 {
    T x, y;
};

template <typename T>
struct Vec3 {
    T x, y, z;
};

struct Pose {
    double x, y, theta;
};