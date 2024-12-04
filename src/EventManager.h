/*
 * File: EventManager.h
 *
 * Description:
 * This file implements the EventManager class, which manages
 * events and their associated callbacks in an event-driven system.
 * It provides functionality for firing events, setting up listeners,
 * and processing events in sequence.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */

#pragma once

// Enumerates all possible events.
typedef enum Event {
    CheckFirst2Dots,         // Check the first two dots.
    SlowDown,                // Event indicating to slow down.
    SpeedUp,                 // Event indicating to speed up.
    Reached5cm,              // Triggered when 5 cm distance is reached.
    Check5cm,                // Check the 5 cm distance.
    Collision,               // Event triggered during a collision.
    TurnLeft,                // Triggered to perform a left turn.
    TurnRight,               // Triggered to perform a right turn.
    TakeLeft,                // Take a left action.
    TakeRight,               // Take a right action.
    CheckFirst3DotsRight,    // Check the first three dots on the right.
    PrepareCollision,        // Prepare for a collision scenario.
    NUMBER_OF_EVENTS         // The total number of events (must remain last).
} Event;

typedef void (*Callback)(Event event); // Function pointer for event callbacks.

/*
 * Manages events and their associated callbacks.
 * Provides functionality for firing events, setting up listeners,
 * and processing events in order.
 */
class EventManager {
    struct Promise {
        bool fired = false;              // Indicates if the event has been triggered.
        Callback callback = nullptr;     // Callback function for the event.
    };

    struct Promise callbackList[NUMBER_OF_EVENTS]; // Array of promises for events.
    int last = 0;                                   // Index of the last processed event.

public:
    /*
     * Constructor for the EventManager.
     * Initializes an empty manager.
     */
    EventManager() {
        // Default initialization.
    }

    /*
     * Fires an event, marking it as triggered.
     *
     * event: The event to fire.
     */
    void fireEvent(Event event) {
        callbackList[event].fired = true;
    }

    /*
     * Sets up a listener for a specific event with a callback function.
     *
     * event: The event to listen for.
     * callback: The function to be called when the event is triggered.
     */
    void setupListener(Event event, Callback callback) {
        callbackList[event].callback = callback;
    }

    /*
     * Cancels all active events by resetting their fired status.
     */
    void cancelAllEvents() {
        for (int i = last; i < NUMBER_OF_EVENTS; i++) {
            callbackList[i].fired = false;
        }
    }

    /*
     * Processes the next triggered event, calling its callback if available.
     *
     * returns: True if an event was processed; false otherwise.
     */
    bool next() {
        for (int i = last; i < NUMBER_OF_EVENTS; i++) {
            if (callbackList[i].fired) {
                callbackList[i].fired = false; // Reset the fired status.
                if (callbackList[i].callback != nullptr) {
                    callbackList[i].callback(Event(i)); // Call the event's callback.
                }
                last += 1; // Move to the next event.
                return true;
            }
        }
        last = 0; // Reset index if no events remain.
        return false;
    }
};
