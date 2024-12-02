#include "RATS.h"
#include "IRSensor.h"
#include "UserInterface.h"
#include "PathFollowing.h"
#include "Odemetry.h"
#include "IntertialMeasurementUnit.h"
#include "EventManager.h"
#include "Queue.h"

#define LOOP for(;;)
#define EVENT(NAME,CODE) eventManager.setupListener(NAME,[](Event e){ CODE });
#define FIRE(NAME) eventManager.fireEvent(NAME);

// Create odometry instance
RobotOdometry odometry = RobotOdometry(WHEEL_DISTANCE, TICKS_PER_REV, WHEEL_DIAMETER);
IntertialMeasurementUnit ratsIMU = IntertialMeasurementUnit();
EventManager eventManager = EventManager();

// Initialize Logger
LogQueue<String> logq = LogQueue<String>();

milliseconds magDebounce = 0;

bool prepareCollision = false;

void setupEvents();

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

void loop() {
	UserInterface::showGoScreen();
    odometry.reset();
    Pololu3piPlus32U4::Encoders::getCountsAndResetLeft();
    Pololu3piPlus32U4::Encoders::getCountsAndResetRight();


    milliseconds sum = 0;
    unsigned long count = 0;

    // milliseconds lag = 5;
    const milliseconds maxTime = 10;
    
    IRSensor::resetPathSignDetector();
    bool eventsPushed = false;
    
    PathFollowing::start();
    PathFollowing::speedUp();
    FIRE(CheckFirst2Dots);
    FIRE(CheckFirst3DotsRight);
    LOOP {
        const milliseconds frameStart = millis();
    
        // run high priority tasks
        IRSensor::scan();
        PathFollowing::follow();
        odometry.update(Pololu3piPlus32U4::Encoders::getCountsLeft(), Pololu3piPlus32U4::Encoders::getCountsRight());
    
        // deal with left over low priority events
        if(eventsPushed) {
            while(eventManager.next());
            eventsPushed = false;
        }

        if (frameStart - magDebounce > MAG_DEBOUNCE_THRESHOLD) { // Debounce to ensure we aren't logging the same anomaly. This may need to be adjusted.
            if (ratsIMU.foundAnamoly().exists()) {
                logq.add("Magnetic Anamoly", odometry.getPose().x, odometry.getPose().y); 
                magDebounce = millis();
            }
        }
        // turn around
        if(IRSensor::isCollisionDetected() && prepareCollision) {
            PathFollowing::stop();
            eventManager.cancelAllEvents();
            logq.add("Collision Detected", odometry.getPose().x, odometry.getPose().y);
            PathFollowing::turnAround();
            PathFollowing::start();
            IRSensor::resetPathSignDetector();
            delay(250);
            PathFollowing::speedUp();

            // found 3!
            while (IRSensor::getRemainingDots() != 3) { 
                IRSensor::scan(); 
                PathFollowing::follow();
            }

            milliseconds current = millis();
            while (IRSensor::getRemainingDots() != 4 && millis() - current < 90) {
                IRSensor::scan();
                PathFollowing::follow();
            }

            if (IRSensor::getRemainingDots() >= 4) {
                while(!IRSensor::seeingRight()) {
                    IRSensor::scan(); 
                    PathFollowing::follow();
                }
                PathFollowing::turnRight();
                IRSensor::resetPathSignDetectorRight();
            } else {
                while(!IRSensor::seeingLeft()) {
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
        // end condition
        if (!PathFollowing::canFollowPath()) {
            eventManager.cancelAllEvents();
            IRSensor::resetPathSignDetector();
            break;
        }
        // deal with low priority events
        while(eventManager.next()) {
            // ok too long, save them for later
            if(millis() - frameStart > maxTime) {
                eventsPushed = true;
                break;
            }
        }
    
        const milliseconds frameTime = millis() - frameStart;
        sum += frameTime;
        count += 1;
    }

	UserInterface::showMessageNotYielding("FPS:" + String((sum / count)* 100) , 2);
	UserInterface::showMessageNotYielding("X:" + String(odometry.getX()), 4);
	UserInterface::showMessage("Y:" + String(odometry.getY()), 5);
	UserInterface::clearScreen();

    LogQueue<String>::Log* currentLog = logq.getFirst();
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
            while (buttonA.isPressed()); // Wait for button release
        }

        if (buttonC.isPressed()) {
            if (currentLog && currentLog->next) {
                currentLog = currentLog->next;
                UserInterface::clearScreen();
            }
            while (buttonC.isPressed()); // Wait for button release
        }

        if (buttonB.isPressed()) {
            break; // Exit log viewing
        }
    }
}

void setupEvents() {

    EVENT(SpeedUp,{
		PathFollowing::speedUp();
		PathFollowing::start();
    })

    {
        static milliseconds t0;
        
        EVENT(SlowDown,{
            PathFollowing::slowDown();
            t0 = millis();
            FIRE(Check5cm);
        })
        
        EVENT(Check5cm,{
            if(millis() - t0 >= 200) {
                FIRE(Reached5cm);
            } else {
                FIRE(Check5cm);
            }
        })
    }

    EVENT(CheckFirst2Dots,{
		if(IRSensor::fastFound2DotsLeft()) {
			FIRE(SlowDown);
		} else {
			FIRE(CheckFirst2Dots);
		}
	})

    EVENT(CheckFirst3DotsRight,{
		if(IRSensor::fastFound3DotsRight()) {
			FIRE(PrepareCollision);
		} else {
			FIRE(CheckFirst3DotsRight);
		}
	})

	EVENT(Reached5cm,{
        PathFollowing::stop();
        delay(300);
		auto orientation = ratsIMU.getOrientation();
        logq.add(
            "SENSOR DATA: Pitch: "+ String(orientation.x) + 
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

    EVENT(PrepareCollision,{
        PathFollowing::slowToSpeed(75);
        prepareCollision = true;
    });
}