/*
 * File: IRSensor.cpp
 *
 * Description:
 * This file implements the IRSensor namespace, which provides functionality
 * for managing infrared sensors and bump sensors. It includes methods for
 * detecting path markers, line tracking, obstacle detection, and sensor
 * calibration.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */

#include "IRSensor.h"


// Constants for the maximum number of sensors and dots.
#define MAX_SIGN_DOTS 5
#define NUM_IRSENSORS 5

/*
 * Enum to represent the positions of IR sensors on the robot.
 * Each sensor has a specific role in navigation and detection.
 */
typedef enum IRS {
    LEFT = 0,           // Sensor on the far left.
    MIDDLE_LEFT = 1,    // Left of the center sensor.
    CENTER = 2,         // Central sensor.
    MIDDLE_RIGHT = 3,   // Right of the center sensor.
    RIGHT = 4,          // Sensor on the far right.
    SENTINEL = 5,       // Boundary marker for sensor array.
} IRSensorAtLocation;

// Sensor objects for line and bump detection.
Pololu3piPlus32U4::LineSensors lineSensors;
Pololu3piPlus32U4::BumpSensors bumpSensors;

// Array to store IR sensor readings.
static uint16_t lineSensorValues[NUM_IRSENSORS];

/*
 * Scanner class:
 * - Detects transitions between black and white surfaces.
 * - Tracks the duration of these surfaces using timestamps.
 */
class Scanner {
public:
    Scanner() : state(WHITE), t0(millis()) {}

    /*
     * Detects transitions between black and white regions.
     * Returns the duration of detected black bars (in milliseconds).
     */
    Option<milliseconds> scan(const bool blackDetected) {
        switch (this->state) {
            case WHITE: {
                if (blackDetected) {
                    this->state = BLACK;
                    this->t0 = millis(); // Start timestamp for black region.
                    return Option<milliseconds>();
                }
                break;
            }
            case BLACK: {
                if (!blackDetected) {
                    this->state = WHITE;
                    const uint64_t t1 = millis();
                    const uint64_t delta = t1 - t0; // Calculate duration.
                    this->t0 = t1;
                    return Option<milliseconds>(delta);
                }
                break;
            }
        }
        return Option<milliseconds>(); // No new value detected.
    }

private:
    /*
     * Enum for current surface state (black or white).
     */
    typedef enum {
        WHITE, // Currently detecting white.
        BLACK  // Currently detecting black.
    } ReadingState;

    ReadingState state; // Current detection state.
    milliseconds t0;    // Timestamp of the current state.
};

/*
 * Template class for scanning path signs at specific sensor positions.
 */
template<IRSensorAtLocation type>
class SignScanner {
public:
    void scan() {
        if (scanner.scan(lineSensorValues[type] > 700).exists()) {
            counts += 1; // Increment count for detected black bars.
        }
    }

    unsigned int getCounts() {
        return counts; // Return the count of detected black bars.
    }

    void reset() {
        counts = 0; // Reset the count.
    }

private:
    mutable Scanner scanner = Scanner();
    unsigned int counts = 0; // Count of detected black bars.
};

// Global instances for specific scanners.
SignScanner<IRSensorAtLocation::LEFT> leftScanner;
SignScanner<IRSensorAtLocation::RIGHT> rightScanner;

/*
 * Initializes the IR sensors by setting their timeout
 * and calibrating the bump sensors.
 */
void IRSensor::initializeIR() {
    lineSensors.setTimeout(IRSENSOR_SAMPLING_TIME);
    bumpSensors.calibrate();
}

/*
 * Resets the detection history of the left path sign scanner.
 */
void IRSensor::resetPathSignDetector() {
    leftScanner.reset();
}

/*
 * Resets the detection history of the right path sign scanner.
 */
void IRSensor::resetPathSignDetectorRight() {
    rightScanner.reset();
}

/*
 * Returns the count of black bars detected on the right path sign scanner.
 */
int IRSensor::getHistory() {
    return rightScanner.getCounts();
}

/*
 * Resets the detection history for the right scanner.
 */
void IRSensor::eraseHistory() {
    rightScanner.reset();
}

/*
 * Detects whether two black dots are detected on the left scanner.
 */
bool IRSensor::fastFound2DotsLeft() {
    return leftScanner.getCounts() >= 2;
}

/*
 * Detects whether three black dots are detected on the right scanner.
 */
bool IRSensor::fastFound3DotsRight() {
    return rightScanner.getCounts() >= 3;
}

/*
 * Returns the count of remaining black dots detected on the left scanner.
 */
IRSensor::Dots IRSensor::getRemainingDots() {
    return leftScanner.getCounts();
}

/*
 * Reads sensor values and updates the scanners.
 */
void IRSensor::scan() {
    lineSensors.readCalibrated(lineSensorValues);
    bumpSensors.read();
    leftScanner.scan();
    rightScanner.scan();
}

/*
 * Returns the calibrated reflectance value for the right sensor.
 */
int IRSensor::reflectanceRight() {
    return lineSensorValues[RIGHT];
}

/*
 * Returns the calibrated reflectance value for the left sensor.
 */
int IRSensor::reflectanceLeft() {
    return lineSensorValues[LEFT];
}

/*
 * Checks if the right sensor is detecting a line.
 */
bool IRSensor::seeingRight() {
    return lineSensorValues[RIGHT] > 800;
}

/*
 * Checks if the left sensor is detecting a line.
 */
bool IRSensor::seeingLeft() {
    return lineSensorValues[LEFT] > 800;
}

/*
 * Checks if the center sensor is detecting a line.
 */
bool IRSensor::seeingCenter() {
    return lineSensorValues[CENTER] > 800;
}

/*
 * Checks if both bump sensors are pressed, indicating a collision.
 */
bool IRSensor::isCollisionDetected() {
    return bumpSensors.leftIsPressed() && bumpSensors.rightIsPressed();
}

/*
 * Calibrates the IR sensors by sweeping over the line and 
 * adjusting the sensor values.
 */
void IRSensor::calibrateIR() {
    using namespace Pololu3piPlus32U4;

    ledRed(true);
    ledYellow(true);

    delay(1000); // Delay before calibration starts.

    // Rotate to sweep sensors over the line.
    Motors::setSpeeds(CALIBRATION_SPEED - 4, CALIBRATION_SPEED);

    while (lineSensorValues[IRSensorAtLocation::CENTER] < 700) {
        lineSensors.calibrate();
        lineSensors.readCalibrated(lineSensorValues);
    }

    while (lineSensorValues[IRSensorAtLocation::CENTER] > 700) {
        lineSensors.calibrate();
        lineSensors.readCalibrated(lineSensorValues);
    }

    milliseconds t0 = millis();
    while (millis() - t0 < 2500) {
        lineSensors.calibrate();
    }

    Motors::setSpeeds(0, 0); // Stop after calibration.
    ledRed(false);
    ledYellow(false);
}


/*
 * Assesses whether the robot's sensors detect the line
 * and calculates the weighted average of the values obtained
 * from the line sensors.
 *
 * Returns LineDetectionResult.
 *
 * If robot's sensors detect the line:
 *   LineDetectionResult will have a value (weighted avg of 3 central IR sensors).
 *
 * If robot's sensors do not detect the line:
 *   LineDetectionResult will be empty.
 */

LineDetectionResult IRSensor::detectLine() {
    bool onLine = false;
    uint32_t avg = 0; // this is for the weighted total
    uint16_t sum = 0; // this is for the denominator, which is <= 64000
    static uint16_t lastPosition = 0;

    lineSensors.readCalibrated(lineSensorValues);

    for (uint8_t i = IRSensorAtLocation::MIDDLE_LEFT; i <= IRSensorAtLocation::MIDDLE_RIGHT; i++) {
        const uint16_t value = lineSensorValues[i];

        // keep track of whether we see the line at all
        if (value > LINE_THRESHOLD) {
            onLine = true;
        }

        // only average in values that are above a noise threshold
        if (value > NOISE_THRESHOLD) {
            avg += static_cast < uint32_t > (value) * (i * 1000);
            sum += value;
        }
    }

    if (!onLine) {
        // if reached end (None of them see line)
        if (sum == 0) {
            return LineDetectionResult();
        }

        // If it last read to the left of center, return 0.
        if (lastPosition < (NUM_IRSENSORS - 3) * 1000 / 2) {
            return LineDetectionResult(0);
        }
        // If it last read to the right of center, return the max.
        return LineDetectionResult((NUM_IRSENSORS - 1) * 1000);
    }

    lastPosition = avg / sum;
    return LineDetectionResult(static_cast < int > (lastPosition));
}
