/*
 * File: Odometry.h
 *
 * Description:
 * This file defines the `RobotOdometry` class, which calculates and updates the
 * robot's position and orientation (odometry) based on encoder tick counts. It
 * uses the robot's physical parameters, such as wheel diameter and wheel distance,
 * to compute movement and rotation in millimeters and radians.
 *
 * Author: OCdt Gratton
 * Version: 2024-12-01
 */

#pragma once

#include "RATS.h"

/**
 * Class for calculating and managing robot odometry.
 * Updates the robot's x, y coordinates and heading (theta) based on
 * encoder readings.
 */
class RobotOdometry {
private:
    const double wheelDistance;          // Distance between the robot's wheels (mm).
    const double ticksPerRevolution;     // Encoder ticks per full wheel revolution.
    const double wheelDiameter;          // Diameter of the robot's wheels (mm).
    const double mmPerTick;              // Distance traveled per encoder tick (mm).

    double x;                            // Current x position of the robot (mm).
    double y;                            // Current y position of the robot (mm).
    double theta;                        // Current orientation of the robot (radians).

    int16_t prevLeft;                    // Previous left encoder reading.
    int16_t prevRight;                   // Previous right encoder reading.

    /**
     * Normalizes an angle to the range [-π, π].
     *
     * @param angle The input angle in radians.
     * @return The normalized angle in radians.
     */
    inline double normalizeAngle(double angle) {
        return atan2(sin(angle), cos(angle));
    }

public:
    /**
     * Constructor to initialize odometry parameters.
     *
     * @param wheelDist Distance between the robot's wheels (default: 86.0 mm).
     * @param ticksPerRev Encoder ticks per wheel revolution (default: 358.3).
     * @param wheelDiam Diameter of the robot's wheels (default: 32.0 mm).
     * @param mmPerTick Distance traveled per encoder tick (default: calculated based on wheel diameter and ticks).
     */
    RobotOdometry(double wheelDist = 86.0, double ticksPerRev = 358.3, double wheelDiam = 32.0,
                  double mmPerTick = (M_PI * 32) / 358.3) :
            wheelDistance(wheelDist),
            ticksPerRevolution(ticksPerRev),
            wheelDiameter(wheelDiam),
            mmPerTick(mmPerTick),
            x(0.0),
            y(0.0),
            theta(0.0),
            prevLeft(0),
            prevRight(0) {}

    /**
     * Resets the robot's odometry to the origin (x=0, y=0, theta=0).
     */
    void reset() {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        prevLeft = 0;
        prevRight = 0;
    }

    /**
     * Updates the robot's position and orientation based on encoder ticks.
     *
     * @param leftTicks Current left encoder tick count.
     * @param rightTicks Current right encoder tick count.
     */
    void update(int32_t leftTicks, int32_t rightTicks) {
        int16_t deltaLeft = leftTicks - prevLeft;         // Change in left encoder ticks.
        int16_t deltaRight = rightTicks - prevRight;      // Change in right encoder ticks.
        double deltaLeftDist = deltaLeft * mmPerTick;     // Distance traveled by the left wheel.
        double deltaRightDist = deltaRight * mmPerTick;   // Distance traveled by the right wheel.
        double deltaCenter = (deltaLeftDist + deltaRightDist) / 2.0; // Average forward distance.
        double deltaTheta = (deltaRightDist - deltaLeftDist) / wheelDistance; // Change in orientation.
        double avgTheta = theta + deltaTheta / 2.0;       // Average orientation during the update.

        // Update x, y position based on forward distance and orientation.
        x += deltaCenter * cos(avgTheta);
        y -= deltaCenter * sin(avgTheta);

        // Update orientation and normalize it.
        theta += deltaTheta;
        theta = normalizeAngle(theta);

        // Update previous encoder readings.
        prevLeft = leftTicks;
        prevRight = rightTicks;
    }

    /**
     * Gets the current x position of the robot.
     *
     * @return The x position in millimeters.
     */
    double getX() const {
        return x;
    }

    /**
     * Gets the current y position of the robot.
     *
     * @return The y position in millimeters.
     */
    double getY() const {
        return y;
    }

    /**
     * Gets the current orientation of the robot.
     *
     * @return The orientation (theta) in radians.
     */
    double getTheta() const {
        return theta;
    }

    /**
     * Gets the current pose (x, y, theta) of the robot.
     *
     * @return A Pose structure containing the robot's position and orientation.
     */
    Pose getPose() const {
        return {x, y, theta};
    }
};
