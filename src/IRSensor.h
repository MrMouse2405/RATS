/*
 * File: IRSensor.h
 *
 * Description:
 * This header file declares the IRSensor namespace, which contains functions
 * and types for managing infrared sensors and bump sensors. These functions
 * include initialization, calibration, line detection, and path marker detection.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */


#pragma once

#include "RATS.h"

namespace IRSensor {

    /*
     * Enum representing the types of path signs detected by the sensors.
     * These can indicate various actions or statuses during navigation.
     */
    typedef enum RST {
        None = 0,               // No path sign detected.
        CalculateElevation = 2, // Signal to calculate elevation.
        TurnRight = 3,          // Signal to turn right.
        TurnLeft = 4,           // Signal to turn left.
        Error = 5,              // Error in detection.
    } PathSignType;

    typedef unsigned int Dots; // Represents the count of detected dots.

    /*
     * Initializes the IR sensors and the bump sensors.
     * - Sets the timeout for IR sensors.
     * - Calibrates the bump sensors to ensure accurate collision detection.
     */
    void initializeIR();

    /*
     * Calibrates the IR sensors by sweeping them over a calibration track.
     * - Adjusts the sensor readings for accurate detection of lines and surfaces.
     */
    void calibrateIR();

    /*
     * Reads and updates the calibrated IR sensor values.
     * - Also updates bump sensor readings.
     * - Used for real-time scanning during navigation.
     */
    void scan();

    /*
     * Resets the detection history of the left path sign scanner.
     * - Clears the count of black dots detected by the left scanner.
     */
    void resetPathSignDetector();

    /*
     * Resets the detection history of the right path sign scanner.
     * - Clears the count of black dots detected by the right scanner.
     */
    void resetPathSignDetectorRight();

    /*
     * Quickly checks if two black dots have been detected by the left scanner.
     * - Returns true if at least two dots are detected, otherwise false.
     */
    bool fastFound2DotsLeft();

    /*
     * Quickly checks if three black dots have been detected by the right scanner.
     * - Returns true if at least three dots are detected, otherwise false.
     */
    bool fastFound3DotsRight();

    /*
     * Checks if the right IR sensor is detecting a line.
     * - Returns true if the right sensor's calibrated value exceeds the threshold.
     */
    bool seeingRight();

    /*
     * Checks if the left IR sensor is detecting a line.
     * - Returns true if the left sensor's calibrated value exceeds the threshold.
     */
    bool seeingLeft();

    /*
     * Checks if the center IR sensor is detecting a line.
     * - Returns true if the center sensor's calibrated value exceeds the threshold.
     */
    bool seeingCenter();

    /*
     * Checks if both bump sensors are pressed, indicating a collision.
     * - Returns true if both sensors are triggered, otherwise false.
     */
    bool isCollisionDetected();

    /*
     * Retrieves the count of remaining dots detected by the left scanner.
     * - Used to determine progress along the path based on left-side markings.
     */
    Dots getRemainingDots();

    /*
     * Retrieves the historical count of black dots detected by the right scanner.
     * - Used to analyze the pattern or path on the right side.
     */
    int getHistory();

    /*
     * Clears the historical count of black dots detected by the right scanner.
     * - Resets the right scanner's detection data.
     */
    void eraseHistory();

    /*
     * Returns the calibrated reflectance value of the right IR sensor.
     * - Reflectance value indicates how much light is reflected back to the sensor.
     */
    int reflectanceRight();

    /*
     * Returns the calibrated reflectance value of the left IR sensor.
     * - Reflectance value indicates how much light is reflected back to the sensor.
     */
    int reflectanceLeft();

    /*
     * Detects the line and calculates the weighted average of reflectance values.
     * - Uses the middle sensors (left, center, right).
     * - Returns the line's position as a `LineDetectionResult`.
     *   If no line is detected, the result will be empty.
     */
    LineDetectionResult detectLine();
}
