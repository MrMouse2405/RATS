#pragma once
#include "RATS.h"

namespace PathFollowing {

    void start();
    void stop();

    void slowDown();
    void slowToSpeed(int speed);
    void speedUp();
    
    bool canFollowPath();
    void follow();

    void turnLeft();
    void turnRight();
    void turnAround();

    int getLeftSpeed();
    int getRightSpeed();
}