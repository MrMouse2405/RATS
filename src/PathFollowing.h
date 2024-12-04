/*
 * File: PathFollowing.h
 *
 * Description:
 * This header file declares the `PathFollowing` namespace, which contains
 * functions for managing the robot's line-following behavior. The namespace
 * includes methods for starting, stopping, turning, and controlling the
 * robot's movement during path navigation.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */

#pragma once

#include "RATS.h"

namespace PathFollowing {

    /**
     * Starts the path-following process.
     * Sets the robot's state to `Following`.
     */
    void start();

    /**
     * Stops the robot and sets the state to `ReachedEnd`.
     */
    void stop();

    /**
     * Reduces the robot's maximum speed.
     */
    void slowDown();

    /**
     * Adjusts the robot's maximum speed to a specified value.
     *
     * @param speed The new maximum speed.
     */
    void slowToSpeed(int speed);

    /**
     * Restores the robot's maximum speed to its default value.
     */
    void speedUp();

    /**
     * Checks if the robot can follow the path.
     *
     * @return True if the robot is actively following the path, otherwise false.
     */
    bool canFollowPath();

    /**
     * Implements the line-following algorithm using a PID controller.
     * Adjusts motor speeds based on the line's position and the robot's error.
     */
    void follow();

    /**
     * Turns the robot to the left.
     */
    void turnLeft();

    /**
     * Turns the robot to the right.
     */
    void turnRight();

    /**
     * Turns the robot 180 degrees to reverse its direction.
     */
    void turnAround();

    /**
     * Retrieves the current speed of the left motor.
     *
     * @return The speed of the left motor.
     */
    int getLeftSpeed();

    /**
     * Retrieves the current speed of the right motor.
     *
     * @return The speed of the right motor.
     */
    int getRightSpeed();
}
