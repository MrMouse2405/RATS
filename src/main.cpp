#include "RATS.h"
#include "IRSensor.h"
#include "UserInterface.h"
#include "PathFollowing.h"
#include "Odemetry.h"
#include "IntertialMeasurementUnit.h"
#include "EventManager.h"

#define LOOP for(;;)
#define EVENT(NAME,CODE) eventManager.setupListener(NAME,[](Event e){ CODE });
#define FIRE(NAME) eventManager.fireEvent(NAME);

// Create odometry instance
RobotOdometry odometry = RobotOdometry(WHEEL_DISTANCE, TICKS_PER_REV, WHEEL_DIAMETER);
IntertialMeasurementUnit ratsIMU = IntertialMeasurementUnit();
EventManager eventManager = EventManager();

void setupEvents();

void setup() {
	IRSensor::initializeIR();
	UserInterface::initializeUI();

	UserInterface::showWelcomeScreen();

	IRSensor::calibrateIR();
	ratsIMU.calibrate();
    setupEvents();
}

void loop() {
	UserInterface::showGoScreen();


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
        // TODO: check magnetic anamoly
        // if (MagneticAnamolyFound) {
        //      eventManager.fireEvent(MagneticAnamoly); 
        // }
        // now make an enum in EventManager.h
        // and put this code in setupEvents();
        // EVENT(yourEvent,{
        //  // your code for magnetic anamoly.
        //  // keep this short and simple
        //  // throw the info in a linkedlist
        /// // or something to process it later, do as u wish.
        // })
        // turn around
        if(IRSensor::isCollisionDetected()) {
            PathFollowing::stop();
            eventManager.cancelAllEvents();
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
				// TODO: Gyroscope
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
}