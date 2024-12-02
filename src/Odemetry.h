// TODO: File, class, and function descriptions
#pragma once
#include "RATS.h"

class RobotOdometry { // TODO: Ecoder overflow handling
private:
    const double wheelDistance;
    const double ticksPerRevolution;
    const double wheelDiameter;
    const double mmPerTick;

    
    double x; // mm
    double y; // mm
    double theta; // radians

    int16_t prevLeft;
    int16_t prevRight;

    inline double normalizeAngle(double angle) {
        return atan2(sin(angle), cos(angle));
    }

public:
    RobotOdometry(double wheelDist = 86.0, double ticksPerRev = 358.3, double wheelDiam = 3.0, double mmPerTick = (M_PI * 32) / 358.3) :
        wheelDistance(wheelDist),
        ticksPerRevolution(ticksPerRev),
        wheelDiameter(wheelDiam),
        mmPerTick(mmPerTick),
        x(0.0),
        y(0.0),
        theta(0.0),
        prevLeft(0),
        prevRight(0)
    {}

    void reset() {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        prevLeft = 0;
        prevRight = 0;
    }

    void update(int32_t leftTicks, int32_t rightTicks) {
        int16_t deltaLeft = leftTicks - prevLeft;
        int16_t deltaRight = rightTicks - prevRight;
        double deltaLeftDist = deltaLeft * mmPerTick;
        double deltaRightDist = deltaRight * mmPerTick;
        double deltaCenter = (deltaLeftDist + deltaRightDist) / 2.0;
        double deltaTheta = (deltaRightDist - deltaLeftDist) / wheelDistance;
        double avgTheta = theta + deltaTheta / 2.0;

        x += deltaCenter * cos(avgTheta);
        y -= deltaCenter * sin(avgTheta);
        theta += deltaTheta;

        theta = normalizeAngle(theta);

        prevLeft = leftTicks;
        prevRight = rightTicks;
    }

    double getX() const {
        return x;
    }
    double getY() const {
        return y;
    }
    double getTheta() const {
        return theta;
    }
    
    Pose getPose() const {
        return {x, y, theta};
    }
};