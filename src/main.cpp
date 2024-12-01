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

void setupEvents();

void setup() {
	IRSensor::initializeIR();
	UserInterface::initializeUI();
    ratsIMU.myIMU.init();
    ratsIMU.myIMU.enableDefault();

	UserInterface::showWelcomeScreen();

	IRSensor::calibrateIR();
	ratsIMU.calibrate();
    // ratsIMU.myIMU.readMag();
    // UserInterface::showMessageTruncate("X "+String(ratsIMU.myIMU.m.x), 0);
    // UserInterface::showMessageTruncate("Y "+String(ratsIMU.myIMU.m.y), 1);
    // UserInterface::showMessageTruncate("Z "+String(ratsIMU.myIMU.m.z), 2);
    setupEvents();
    while (1);
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
    FIRE(CheckFirst2Dots);
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
            if (ratsIMU.foundAnamoly().exists()) { // TODO: Fix this lol
                logq.add("Magnetic Anamoly", odometry.getPose().x, odometry.getPose().y); 
                magDebounce = millis();
            }
        }
        // turn around
        if(IRSensor::isCollisionDetected()) {
            PathFollowing::stop();
            eventManager.cancelAllEvents();
            logq.add("Collision Detected", odometry.getPose().x, odometry.getPose().y);
            PathFollowing::turnAround();
            PathFollowing::start();
            PathFollowing::speedUp();
            FIRE(CheckFirst2Dots);
        }	
        // end condition
        if (!PathFollowing::canFollowPath()) {
            eventManager.cancelAllEvents();
            // check just in case
            PathFollowing::start();
            IRSensor::scan();
            PathFollowing::follow();
            if (PathFollowing::canFollowPath()) {
                FIRE(CheckFirst2Dots);
                continue;
            }
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


    // Scroll through the log using button A and C
    LogQueue<String>::Log* currentLog = logq.getFirst();
    while (true) {
        if (currentLog) {
            String logMessage = "Log: " + currentLog->type;
            int line = 1;
            for (size_t i = 0; i < logMessage.length(); i += 20) {
                UserInterface::showMessageTruncate(logMessage.substring(i, i + 20), line);
                line += 1;
            }
            UserInterface::showMessageTruncate("X: " + String(currentLog->x / 10), line);
            UserInterface::showMessageTruncate("Y: " + String(currentLog->y / 10), line + 1);
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
            if(millis() - t0 > 350) {
                FIRE(Reached5cm);
            } else {
                FIRE(Check5cm);
            }
        })
    }

    EVENT(CheckFirst2Dots,{
		if(IRSensor::fastFound2Dots()) {
			FIRE(SlowDown);
		} else {
			FIRE(CheckFirst2Dots);
		}
	})


	EVENT(Reached5cm,{
		int remaining = IRSensor::getRemainingDots();
        IRSensor::resetPathSignDetector();
        // stop, get some rest, stop running around for a while
        PathFollowing::stop();
        PathFollowing::follow();
        PathFollowing::start();
		switch(remaining) {
			case 0: {
				auto orientation = ratsIMU.getOrientation();
                logq.add("SENSOR DATA: Pitch: "+ String(orientation.x) + 
                " Roll: " + String(orientation.y)
                + " Reflectance Left: " + String(IRSensor::reflectanceLeft()) +
                " Reflectance Right: " + String(IRSensor::reflectanceRight()),
                odometry.getPose().x, odometry.getPose().y);
				delay(100);
				FIRE(CheckFirst2Dots);
				break;
			}
			case 1: {
                if (IRSensor::getHistory() == 3) {
				    FIRE(TurnLeft);
                }
                else {
                    FIRE(TurnRight);
                }
				break;
			}
			case 2: {
				FIRE(TurnRight);
				break;
			}
			default:
				break;
		}
        IRSensor::eraseHistory();
        FIRE(SpeedUp);
	});

	EVENT(TurnLeft,{
        if (IRSensor::getRemainingDots() == 1) {
            eventManager.fireEvent(TurnRight);
        }
		else if(!IRSensor::seeingLeft()) {
			eventManager.fireEvent(TurnLeft);
		}
        else {
            PathFollowing::turnLeft();
            eventManager.fireEvent(CheckFirst2Dots);
        }
	})

	EVENT(TurnRight,{
		if(!IRSensor::seeingRight()) {
			eventManager.fireEvent(TurnRight);
		}
        else {
            PathFollowing::turnRight();
            eventManager.fireEvent(CheckFirst2Dots);
        }
	})
    
    EVENT(MagneticAnamoly,{
        
       
    })

    EVENT(SensorEvent,{
        
        
    })

    EVENT(CollisionEvent,{
        
    })
}