/*
 * File: PathFollowing.cpp
 *
 * Description:
 * This file implements the `PathFollowing` namespace, which provides
 * functionality for robot line-following behavior. It includes methods
 * for starting and stopping the path-following process, turning the robot,
 * and managing speed adjustments. The PID algorithm is used for precise
 * line-following navigation.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */

#include "PathFollowing.h"
#include "IRSensor.h"

/**
 * Namespace for path-following functionality, including state management,
 * motor control, and PID-based line following.
 */
namespace PathFollowing {

    typedef enum PFStates {

        Ready,
        Following,
        ReachedEnd,

    } PathFollowerStates;

    PathFollowerStates state = Ready;

    int maxSpeed = MAX_SPEED;
    int leftSpeed = 0;
    int rightSpeed = 0;
}

/**
 * Checks if the robot is in a state to follow the path.
 *
 * @return True if the robot is actively following the path, otherwise false.
 */
bool PathFollowing::canFollowPath() {
    return state == Following;
}

/**
    * Starts the path-following process.
    * Sets the robot's state to `Following`.
    */
void PathFollowing::start() {
    state = Following;
}

/**
 * Stops the robot and sets the state to `ReachedEnd`.
 */
void PathFollowing::stop() {
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
    state = ReachedEnd;
}

/**
 * Retrieves the current speed of the left motor.
 *
 * @return The speed of the left motor.
 */
int PathFollowing::getLeftSpeed() {
    return leftSpeed;
}

/**
 * Retrieves the current speed of the right motor.
 *
 * @return The speed of the right motor.
 */
int PathFollowing::getRightSpeed() {
    return rightSpeed;
}

/**
 * Turns the robot to the left.
 */
void PathFollowing::turnLeft() {
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(-MAX_SPEED, MAX_SPEED);
    delay(110);
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED - 50, MAX_SPEED);
    milliseconds startTime = millis();
    while (millis() - startTime < 300) {
        while (millis() - startTime < 150) {}
        if (IRSensor::detectLine().exists() && IRSensor::detectLine().get() > 2000) {
            break;
        }
    }
}

/**
 * Turns the robot to the right.
 */
void PathFollowing::turnRight() {
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED, -MAX_SPEED);
    delay(110);
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED, MAX_SPEED - 50);
    milliseconds startTime = millis();
    while (millis() - startTime < 300) {
        while (millis() - startTime < 150) {}
        if (IRSensor::detectLine().exists() && IRSensor::detectLine().get() > 2000) {
            break;
        }
    }
}

/**
 * Turns the robot 180 degrees to reverse its direction.
 */
void PathFollowing::turnAround() {
    Pololu3piPlus32U4::Motors::setSpeeds(-50, -50);
    delay(2);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED, -MAX_SPEED);
    delay(190);
    Pololu3piPlus32U4::Motors::setSpeeds(-50, -50);
    delay(100);
    IRSensor::resetPathSignDetector();
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0);
}

/**
    * Adjusts the robot's maximum speed to a specified value.
    *
    * @param speed The new maximum speed.
    */
void PathFollowing::slowToSpeed(int speed) {
    maxSpeed = speed;
}

/**
  * Restores the robot's maximum speed to its default value.
  */
void PathFollowing::speedUp() {
    maxSpeed = MAX_SPEED;
}

/**
 * Reduces the robot's maximum speed.
 */
void PathFollowing::slowDown() {
    maxSpeed = SLOW_MAX_SPEED;
}

/**
 *
 * Algorithm for following the line
 *
 * sets state to 'ReachedEnd' if the
 * IR sensors no longer detect the line.
 *
 * uses PID controller for moving the robot
 *
 */
void PathFollowing::follow() {

    static int lastError = 0;


    if (state != Following) {
        return;
    }

    LineDetectionResult result = IRSensor::detectLine();
    if (!result.exists()) {
        stop();
        return;
    }

    const int position = result.get();

    /**
     * Our "error" is how far we are away from the center of the
     * line, which corresponds to position 2000.
    */

    const int error = position - 2000;

    /** 
     * Get motor speed difference using PROPORTIONAL_CONSTANT and derivative
     * PID terms (the integral term is generally not very useful
     * for line following). 
    */

    const int speedDifference = error * PROPORTIONAL_CONSTANT / 256 + (error - lastError) * DERIVATIVE_CONSTANT / 256;
    lastError = error;

    /** 
     * Get individual motor speeds.  The sign of speedDifference
     * determines if the robot turns left or right.
    */

    int leftSpeed = maxSpeed + speedDifference;
    int rightSpeed = maxSpeed - speedDifference;

    /**
     * Constrain our motor speeds to be between 0 and MAX_SPEED.
     * One motor will always be turning at MAX_SPEED, and the other
     * will be at MAX_SPEED-|speedDifference| if that is positive,
     * else it will be stationary.  For some applications, you
     * might want to allow the motor speed to go negative so that
     * it can spin in reverse.
    */

    leftSpeed = constrain(leftSpeed, MIN_SPEED, (int16_t) maxSpeed);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, (int16_t) maxSpeed);

    PathFollowing::leftSpeed = leftSpeed;
    PathFollowing::rightSpeed = rightSpeed;

    // Zoom
    Pololu3piPlus32U4::Motors::setSpeeds(leftSpeed, rightSpeed);
}