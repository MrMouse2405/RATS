#pragma once

typedef enum ES
{
    CheckFirst2Dots,
    SlowDown,
    SpeedUp,
    Reached5cm,
    Check5cm,
    Collision,
    TurnLeft,
    TurnRight,
    TakeLeft,
    TakeRight,
    NUMBER_OF_EVENTS, // ALWAYS LAST
} Event;

typedef void (*Callback)(Event event);


class EventManager
{

    struct Promise {
        bool fired = false;
        Callback callback = nullptr;
    };

    struct Promise callbackList[NUMBER_OF_EVENTS];
    int last = 0;
public:
    EventManager()
    {
    }

    void fireEvent(Event event)
    {
        callbackList[event].fired = true;
    }

    void setupListener(Event event, Callback callback)
    {
        callbackList[event].callback = callback;
    }

    void cancelAllEvents() {
        for (int i = last; i < NUMBER_OF_EVENTS; i++) {
            callbackList[i].fired = false;
        }
    }

    bool next() {
        for (int i = last; i < NUMBER_OF_EVENTS; i++) {
            if (callbackList[i].fired) {
                callbackList[i].fired = false;
                if (callbackList[i].callback != nullptr) {
                    callbackList[i].callback(Event(i));
                }
                last += 1;
                return true;
            }
        }
        last = 0;
        return false;
    }
};