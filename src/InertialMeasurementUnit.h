#pragma once

#include "RATS.h"
#include "Pololu3piPlus32U4IMU.h"

/*
 * File: IntertialMeasurementUnit.h
 *
 * Description:
 * This file defines the InertialMeasurementUnit class, which provides
 * calibration and data processing for accelerometer and magnetometer
 * sensors. It includes methods for detecting magnetic anomalies, 
 * calculating orientation (pitch and roll), and handling sensor offsets.
 *
 * Author: OCdt Gratton
 * Version: 2024-12-01
 */

class IntertialMeasurementUnit {
private:
    float xOffset;       // Calibration offset for x-axis magnetic field.
    float yOffset;       // Calibration offset for y-axis magnetic field.
    float zOffset;       // Calibration offset for z-axis magnetic field.
    float pitchOffset;   // Calibration offset for pitch angle.
    float rollOffset;    // Calibration offset for roll angle.

public:
    Pololu3piPlus32U4::IMU myIMU; // IMU sensor object to interface with hardware.

    /*
     * Constructor to initialize offsets to zero.
     */
    IntertialMeasurementUnit()
            : xOffset(0.0), yOffset(0.0), zOffset(0.0),
              pitchOffset(0.0), rollOffset(0.0) {}

    /*
     * Calibrates the IMU by reading magnetometer and accelerometer data.
     * - Determines offsets for magnetic fields and orientation (pitch and roll).
     */
    void calibrate() {
        myIMU.readMag(); // Read magnetometer data.
        myIMU.readAcc(); // Read accelerometer data.

        // Record magnetic field offsets.
        xOffset = myIMU.m.x;
        yOffset = myIMU.m.y;
        zOffset = myIMU.m.z;

        // Calculate normalized accelerometer values.
        float accelX = myIMU.a.x;
        float accelY = myIMU.a.y;
        float accelZ = myIMU.a.z;
        float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        float normX = accelX / magnitude;
        float normY = accelY / magnitude;
        float normZ = accelZ / magnitude;

        // Compute pitch and roll offsets.
        pitchOffset = calculatePitch(normX, normY, normZ);
        rollOffset = calculateRoll(normX, normY, normZ);
    }

    /*
     * Detects a magnetic anomaly by comparing current magnetic strength to a threshold.
     * Returns an optional vector containing the anomaly's position if detected.
     */
    Option<Vec3<float>> foundAnamoly() {
        myIMU.readMag(); // Read magnetometer data.
        int16_t magX = myIMU.m.x;
        int16_t magY = myIMU.m.y;
        int16_t magZ = myIMU.m.z;

        // Calculate magnetic strength with calibration offsets applied.
        float magneticStrength = calculateMagneticStrength(
                magX - xOffset, magY - yOffset, magZ - zOffset);

        // Check against the magnetic threshold.
        if (magneticStrength <= MAG_THRESHOLD) {
            return Option<Vec3<float>>(); // No anomaly detected.
        }

        // Return the anomaly location vector.
        return Option<Vec3<float>>({magX - xOffset, magY - yOffset, magZ - zOffset});
    }

    /*
     * Computes and returns the current magnetic field strength.
     */
    float getStrength() {
        myIMU.readMag(); // Read magnetometer data.
        int16_t magX = myIMU.m.x;
        int16_t magY = myIMU.m.y;
        int16_t magZ = myIMU.m.z;

        // Calculate and return the magnetic strength.
        return calculateMagneticStrength(magX - xOffset, magY - yOffset, magZ - zOffset);
    }

    /*
     * Helper function to calculate the magnitude of the magnetic field vector.
     * - x, y, z: Magnetic field components.
     * Returns the scalar magnitude.
     */
    inline float calculateMagneticStrength(int16_t x, int16_t y, int16_t z) {
        return sqrt((float) x * x + (float) y * y + (float) z * z);
    }

    /*
     * Calculates and returns the current orientation (pitch and roll) in degrees.
     * Uses accelerometer data and applies calibration offsets.
     */
    Vec2<float> getOrientation() {
        myIMU.readAcc(); // Read accelerometer data.

        // Normalize accelerometer values.
        float accelX = myIMU.a.x;
        float accelY = myIMU.a.y;
        float accelZ = myIMU.a.z;
        float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        float normX = accelX / magnitude;
        float normY = accelY / magnitude;
        float normZ = accelZ / magnitude;

        // Calculate pitch and roll angles with offsets applied.
        float pitch = calculatePitch(normX, normY, normZ) - pitchOffset;
        float roll = calculateRoll(normX, normY, normZ) - rollOffset;

        return {pitch, roll}; // Return orientation as a 2D vector.
    }

    /*
     * Helper function to calculate the pitch angle in degrees.
     * - x, y, z: Normalized accelerometer values.
     * Returns the pitch angle.
     */
    inline float calculatePitch(float x, float y, float z) {
        return atan2(-x, sqrt(y * y + z * z)) * 180.0 / M_PI;
    }

    /*
     * Helper function to calculate the roll angle in degrees.
     * - x, y, z: Normalized accelerometer values.
     * Returns the roll angle.
     */
    inline float calculateRoll(float x, float y, float z) {
        return atan2(y, sqrt(x * x + z * z)) * 180.0 / M_PI;
    }
};