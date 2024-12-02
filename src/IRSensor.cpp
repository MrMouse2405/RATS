#include "IRSensor.h"

#define MAX_SIGN_DOTS 5
#define NUM_IRSENSORS 5

typedef enum IRS {
    LEFT = 0, // sensor 1
    MIDDLE_LEFT = 1,
    CENTER = 2,
    MIDDLE_RIGHT = 3,
    RIGHT = 4, // sensor 5
    SENTINEL = 5,
} 
IRSensorAtLocation;

Pololu3piPlus32U4::LineSensors lineSensors;
Pololu3piPlus32U4::BumpSensors bumpSensors;

static uint16_t lineSensorValues[NUM_IRSENSORS];


class Scanner {
public:
    /**
     *
     * Scans the barcode width from the time of
     * initialization of this object
     *
     */
    Scanner() {
        this->state = WHITE;
        this->t0 = millis();
    }

    /**
     *
     * Scans and returns everytime a new "black" bar is detected.
     * returns milliseconds => how long is the width of that bar
     * 
     * Option<milliseconds> will be empty if a new value is not detected.
     * It will contain a value if a new value is detected.
     *
     */
    Option<milliseconds> scan(const bool blackDetected) {
        switch (this->state) {
            case WHITE: {
                if (blackDetected) {
                    this->state = BLACK;
                    this->t0 = millis();
                    return Option<milliseconds>();
                }
                break;
            }
            case BLACK: {
                if (!blackDetected) {
                    this->state = WHITE;
                    const uint64_t t1 = millis();
                    const uint64_t delta = t1 - t0;
                    this->t0 = t1;
                    return Option<milliseconds>(delta);
                }
                break;
            }
        }

        return Option<milliseconds>();
    }
    
private:
    typedef enum {
        // currently seeing white
        WHITE,
        // currently seeing black
        BLACK
    } ReadingState;

    // what are we currently seeing?
    ReadingState state;
    // since when did we start seeing our ReadingState
    milliseconds t0;
};


template<IRSensorAtLocation type>
class SignScanner {
public:
    void scan() {
        if(scanner.scan(lineSensorValues[type] > 500).exists()) {
            counts += 1;
        }
    }
    unsigned int getCounts() {
        return counts;
    }
    void reset() {
        counts = 0;
    }
private:
    mutable Scanner scanner = Scanner();
    unsigned int counts = 0;
};


SignScanner<IRSensorAtLocation::LEFT> leftScanner;
SignScanner<IRSensorAtLocation::RIGHT> rightScanner;


void IRSensor::initializeIR() {
    lineSensors.setTimeout(IRSENSOR_SAMPLING_TIME);
    bumpSensors.calibrate();
}

void IRSensor::resetPathSignDetector() {
    leftScanner.reset();
}

int IRSensor::getHistory() {
    return rightScanner.getCounts();
}

void IRSensor::eraseHistory() {
    rightScanner.reset();
}

bool IRSensor::fastFound2Dots() {

    if (leftScanner.getCounts() >= 2) {
        return true;
    }

    return false;
}

IRSensor::Dots IRSensor::getRemainingDots() {
    return leftScanner.getCounts();
}

void IRSensor::scan() {
    lineSensors.readCalibrated(lineSensorValues);
    bumpSensors.read();
    leftScanner.scan();
    rightScanner.scan();
}

int IRSensor::reflectanceRight() {
    return lineSensorValues[RIGHT];
}

int IRSensor::reflectanceLeft() {
    return lineSensorValues[LEFT];
}


bool IRSensor::seeingRight() {
    return lineSensorValues[RIGHT] > 800;
}

bool IRSensor::seeingLeft() {
    return lineSensorValues[LEFT] > 800;
}

bool IRSensor::isCollisionDetected() {
    return bumpSensors.leftIsPressed() and bumpSensors.rightIsPressed();
}

void IRSensor::calibrateIR() {
    using namespace Pololu3piPlus32U4;

    ledRed(true);
    ledYellow(true);

    // Wait 1 second and then begin automatic sensor calibration
    // by rotating in place to sweep the sensors over the line
    delay(1000);

    // turn left
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

    // stop
    Motors::setSpeeds(0, 0);
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

LineDetectionResult IRSensor::detectLineBiasedLeft() {
    return LineDetectionResult();
}

LineDetectionResult IRSensor::detectLineBiasedRight() {
    return LineDetectionResult();
}