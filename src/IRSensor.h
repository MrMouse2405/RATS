#pragma once
#include "RATS.h"

namespace IRSensor {


    typedef enum RST {
        None = 0,
        CalculateElevation = 2,
        TurnRight = 3,
        TurnLeft = 4,
        Error=5,
    } PathSignType;

    typedef unsigned int Dots;
    
    void initializeIR();

    void calibrateIR();
    void scan();

    void resetPathSignDetector();
    bool fastFound2Dots();
    bool seeingRight();
    bool seeingLeft();

    bool isCollisionDetected();
    
    Dots getRemainingDots();

    int getHistory();
    void eraseHistory();

    LineDetectionResult detectLine();
    LineDetectionResult detectLineBiasedLeft();
    LineDetectionResult detectLineBiasedRight();
}