/*
 * File: main.cpp
 *
 * Description:
 * This is the main control file for the Rolling Autonomous Terrain Sensor (RATS).
 * It initializes the robot's components, sets up event-driven behavior, and
 * implements the main loop for navigation, obstacle detection, and data logging.
 *
 * Key Features:
 * - Event-driven architecture for efficient task management.
 * - Integration with IR sensors, odometry, and IMU for navigation and logging.
 * - Real-time path following with collision avoidance and anomaly detection.
 * - User interface for displaying logs and runtime information.
 *
 * Author: OCdt Syed & OCdt Gratton
 * Version: 2024-1201
 */

#include "RATS.h"
#include "IRSensor.h"
#include "UserInterface.h"
#include "PathFollowing.h"
#include "Odometry.h"
#include "InertialMeasurementUnit.h"
#include "EventManager.h"
#include "Queue.h"

#define LOOP for(;;)
#define EVENT(NAME, CODE) eventManager.setupListener(NAME,[](Event e){ CODE });
#define FIRE(NAME) eventManager.fireEvent(NAME);

/**
 * Global Objects:
 * - Odometry: Tracks the robot's position and orientation.
 * - IMU: Measures pitch and roll for navigation and logging.
 * - EventManager: Manages event-driven behavior and scheduling.
 * - LogQueue: Stores event logs with associated positional data.
 */
RobotOdometry odometry = RobotOdometry(WHEEL_DISTANCE, TICKS_PER_REV, WHEEL_DIAMETER);
IntertialMeasurementUnit ratsIMU = IntertialMeasurementUnit();
EventManager eventManager = EventManager();
LogQueue<String> logq = LogQueue<String>();

// Debounce timer for magnetic anomaly detection.
milliseconds magDebounce = 0;

// Flag for preparing collision avoidance.
bool prepareCollision = false;

// Function declarations.
void setupEvents();

/**
 * Initialization routine for the robot.
 * - Sets up IR sensors, IMU, and user interface.
 * - Calibrates sensors and initializes events.
 */
void setup() {
    IRSensor::initializeIR();
    UserInterface::initializeUI();
    Wire.begin();
    ratsIMU.myIMU.init();
    ratsIMU.myIMU.enableDefault();

    UserInterface::showWelcomeScreen();

    IRSensor::calibrateIR();
    ratsIMU.calibrate();

    setupEvents();
}

/**
 * Main control loop for the robot.
 * - Executes high-priority tasks like scanning sensors and path following.
 * - Handles events and logs data in real-time.
 * - Implements collision detection and recovery logic.
 */
void loop() {
    UserInterface::showGoScreen();
    odometry.reset();
    Pololu3piPlus32U4::Encoders::getCountsAndResetLeft();
    Pololu3piPlus32U4::Encoders::getCountsAndResetRight();

    milliseconds sum = 0;
    unsigned long count = 0;

    IRSensor::resetPathSignDetector();
    bool eventsPushed = false;

    const milliseconds maxTime = 10; // Maximum frame time for low-priority tasks.

    PathFollowing::start();
    PathFollowing::speedUp();

    FIRE(CheckFirst2Dots);
    FIRE(CheckFirst3DotsRight);

    LOOP {
        const milliseconds frameStart = millis();

        // High-priority tasks: sensor scanning, path following, odometry updates.
        IRSensor::scan();
        PathFollowing::follow();
        odometry.update(Pololu3piPlus32U4::Encoders::getCountsLeft(), Pololu3piPlus32U4::Encoders::getCountsRight());

        // Handle queued events.
        if (eventsPushed) {
            while (eventManager.next());
            eventsPushed = false;
        }

        // Magnetic anomaly detection with debounce.
        if (frameStart - magDebounce > MAG_DEBOUNCE_THRESHOLD) {
            if (ratsIMU.foundAnamoly().exists()) {
                logq.add("Magnetic Anomaly", odometry.getPose().x, odometry.getPose().y);
                magDebounce = millis();
            }
        }

        // Collision detection and recovery logic.
        if (IRSensor::isCollisionDetected() && prepareCollision) {
            PathFollowing::stop();
            eventManager.cancelAllEvents();
            logq.add("Collision Detected", odometry.getPose().x, odometry.getPose().y);
            PathFollowing::turnAround();
            PathFollowing::start();
            IRSensor::resetPathSignDetector();
            delay(250);
            PathFollowing::speedUp();

            // Handle path markers during recovery.
            while (IRSensor::getRemainingDots() != 3) {
                IRSensor::scan();
                PathFollowing::follow();
            }

            milliseconds current = millis();
            while (IRSensor::getRemainingDots() != 4 && millis() - current < 90) {
                IRSensor::scan();
                PathFollowing::follow();
            }

            // Choose turn direction based on detected markers.
            if (IRSensor::getRemainingDots() >= 4) {
                while (!IRSensor::seeingRight()) {
                    IRSensor::scan();
                    PathFollowing::follow();
                }
                PathFollowing::turnRight();
                IRSensor::resetPathSignDetectorRight();
            } else {
                while (!IRSensor::seeingLeft()) {
                    IRSensor::scan();
                    PathFollowing::follow();
                }
                PathFollowing::turnLeft();
                IRSensor::resetPathSignDetectorRight();
            }
            prepareCollision = false;
            IRSensor::resetPathSignDetector();
            FIRE(CheckFirst2Dots);
            FIRE(CheckFirst3DotsRight);
        }

        // End condition: stop if the robot cannot follow the path.
        if (!PathFollowing::canFollowPath()) {
            eventManager.cancelAllEvents();
            IRSensor::resetPathSignDetector();
            break;
        }

        // Process low-priority events if time allows.
        while (eventManager.next()) {
            if (millis() - frameStart > maxTime) {
                eventsPushed = true;
                break;
            }
        }

        // Record frame timing.
        const milliseconds frameTime = millis() - frameStart;
        sum += frameTime;
        count += 1;
    }

    // Display runtime data and logs after the loop ends.
    UserInterface::showMessageNotYielding("FPS:" + String((sum / count) * 100), 2);
    UserInterface::showMessageNotYielding("X:" + String(odometry.getX()), 4);
    UserInterface::showMessage("Y:" + String(odometry.getY()), 5);
    UserInterface::clearScreen();

    // Log viewing interface.
    LogQueue<String>::Log *currentLog = logq.getFirst();
    while (true) {
        if (currentLog) {
            String logMessage = "Log: " + currentLog->type;
            int line = 1;
            for (size_t i = 0; i < logMessage.length(); i += 20) {
                UserInterface::showMessageTruncate(logMessage.substring(i, i + 20), line);
                line += 1;
            }
            UserInterface::showMessageTruncate("Y: " + String(currentLog->x / 1000), line);
            UserInterface::showMessageTruncate("X: " + String(currentLog->y / 1000), line + 1);
        } else {
            UserInterface::showMessageTruncate("No Logs Available", 0);
        }

        Pololu3piPlus32U4::ButtonA buttonA;
        Pololu3piPlus32U4::ButtonB buttonB;
        Pololu3piPlus32U4::ButtonC buttonC;
        if (buttonA.isPressed()) {
            if (currentLog && currentLog->prev) {
                currentLog = currentLog->prev;
                UserInterface::clearScreen();
            }
            while (buttonA.isPressed());
        }

        if (buttonC.isPressed()) {
            if (currentLog && currentLog->next) {
                currentLog = currentLog->next;
                UserInterface::clearScreen();
            }
            while (buttonC.isPressed());
        }

        if (buttonB.isPressed()) {
            break;
        }
    }
}

/**
 * Sets up event listeners and associated behaviors.
 * - Includes events for speed control, collision handling, and logging.
 */
void setupEvents() {
    // [Event logic as in the provided code.]

    EVENT(SpeedUp, {
        PathFollowing::speedUp();
        PathFollowing::start();
    })

    {
        static milliseconds t0;

        EVENT(SlowDown, {
            PathFollowing::slowDown();
            t0 = millis();
            FIRE(Check5cm);
        })

        EVENT(Check5cm, {
            if (millis() - t0 >= 200) {
                FIRE(Reached5cm);
            } else {
                FIRE(Check5cm);
            }
        })
    }

    EVENT(CheckFirst2Dots, {
        if (IRSensor::fastFound2DotsLeft()) {
            FIRE(SlowDown);
        } else {
            FIRE(CheckFirst2Dots);
        }
    })

    EVENT(CheckFirst3DotsRight, {
        if (IRSensor::fastFound3DotsRight()) {
            FIRE(PrepareCollision);
        } else {
            FIRE(CheckFirst3DotsRight);
        }
    })

    EVENT(Reached5cm, {
        PathFollowing::stop();
        delay(300);
        auto orientation = ratsIMU.getOrientation();
        logq.add(
                "SENSOR DATA: Pitch: " + String(orientation.x) +
                " Roll: " + String(orientation.y) +
                " Reflectance Left: " + String(IRSensor::reflectanceLeft()) +
                " Reflectance Right: " + String(IRSensor::reflectanceRight()
                ),
                odometry.getPose().x,
                odometry.getPose().y
        );

        PathFollowing::start();
        PathFollowing::speedUp();
        FIRE(CheckFirst2Dots);
        IRSensor::resetPathSignDetector();
    });

    EVENT(PrepareCollision, {
        PathFollowing::slowToSpeed(75);
        prepareCollision = true;
    });
}