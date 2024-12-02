#include "PathFollowing.h"
#include "IRSensor.h"

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

bool PathFollowing::canFollowPath() {
    return state == Following;
}

void PathFollowing::start() {
    state = Following;
}

void PathFollowing::stop() {
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    state = ReachedEnd;
}

int PathFollowing::getLeftSpeed() {
    return leftSpeed;
}

int PathFollowing::getRightSpeed() {
    return rightSpeed;
}

void PathFollowing::turnLeft() {
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(-MAX_SPEED,MAX_SPEED);
    delay(110);
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    delay(20);
    // while(!IRSensor::detectLine().exists()) {
    //     IRSensor::scan();
    // }
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED-50,MAX_SPEED);
    milliseconds startTime = millis();
    while (millis() - startTime < 300) {
        while (millis() - startTime < 150) {}
        if (IRSensor::detectLine().exists() && IRSensor::detectLine().get() > 2000) {
            break;
        }
    }
}

// void PathFollowing::turnRight() {
//     Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED,0);
//     delay(200);
//     while(!IRSensor::detectLine().exists()) {
//         IRSensor::scan();
//     }
//     Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED,MAX_SPEED);
// }

void PathFollowing::turnRight() {
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    delay(20);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED,-MAX_SPEED);
    delay(110);
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    delay(20);
    // while(!IRSensor::detectLine().exists()) {
    //     IRSensor::scan();
    // }
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED,MAX_SPEED-50);
    milliseconds startTime = millis();
    while (millis() - startTime < 300) {
        while (millis() - startTime < 150) {}
        if (IRSensor::detectLine().exists() && IRSensor::detectLine().get() > 2000) {
            break;
        }
    }
}


void PathFollowing::turnAround() {
    Pololu3piPlus32U4::Motors::setSpeeds(-50,-50);
    delay(2);
    Pololu3piPlus32U4::Motors::setSpeeds(MAX_SPEED,-MAX_SPEED);
    delay(190);
    Pololu3piPlus32U4::Motors::setSpeeds(-50,-50);
    delay(100);
    IRSensor::resetPathSignDetector();
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
}

void PathFollowing::slowToSpeed(int speed) {
    maxSpeed = speed;
}

void PathFollowing::speedUp() {
    maxSpeed = MAX_SPEED;
}

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
    
    leftSpeed = constrain(leftSpeed, MIN_SPEED, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, (int16_t)maxSpeed);
   
    PathFollowing::leftSpeed = leftSpeed;
    PathFollowing::rightSpeed = rightSpeed;

    // Zoom
    Pololu3piPlus32U4::Motors::setSpeeds(leftSpeed, rightSpeed);
}
