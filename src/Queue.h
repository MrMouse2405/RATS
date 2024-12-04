#pragma once

/*
 * File: LogQueue.h
 *
 * Description:
 * This file defines the LogQueue class, a template-based implementation
 * of a doubly linked list queue for storing logs. Each log contains a type,
 * x and y coordinates, and pointers to adjacent logs. The class provides
 * methods for adding logs, retrieving the first/last log, checking queue size,
 * and clearing the queue.
 *
 * Author: OCdt Gratton
 * Version: 2024-12-01
 */

/**
 * A template-based doubly linked list queue designed to store logs.
 * Each log contains a type, x and y coordinates, and pointers to
 * adjacent logs.
 *
 * @tparam T The data type of the log's type field.
 */
template<typename T>
class LogQueue {
public:
    /**
     * Struct representing a single log entry in the queue.
     */
    struct Log {
        T type;        // The type or category of the log entry.
        double x;      // The x-coordinate associated with the log.
        double y;      // The y-coordinate associated with the log.
        Log *prev;     // Pointer to the previous log in the queue.
        Log *next;     // Pointer to the next log in the queue.

        /**
         * Constructor to initialize a log entry.
         *
         * @param value The type of the log entry.
         * @param xPos The x-coordinate for the log entry.
         * @param yPos The y-coordinate for the log entry.
         */
        Log(const T &value, double xPos, double yPos)
                : type(value), x(xPos), y(yPos), prev(nullptr), next(nullptr) {}
    };

    Log *head;        // Pointer to the first log in the queue.
    Log *tail;        // Pointer to the last log in the queue.
    int queueSize;    // Tracks the current size of the queue.

    /**
     * Constructor to initialize an empty log queue.
     */
    LogQueue() : head(nullptr), tail(nullptr), queueSize(0) {}

    /**
     * Adds a new log entry to the end of the queue.
     *
     * @param value The type of the log entry.
     * @param xPos The x-coordinate for the log entry.
     * @param yPos The y-coordinate for the log entry.
     */
    void add(const T &value, double xPos, double yPos) {
        Log *newLog = new Log(value, xPos, yPos);

        // If the queue is empty, set the new log as the head and tail.
        if (!head) {
            head = tail = newLog;
        } else {
            tail->next = newLog;  // Link the current tail to the new log.
            newLog->prev = tail;  // Link the new log to the current tail.
            tail = newLog;        // Update the tail to the new log.
        }

        queueSize++; // Increment the queue size.
    }

    /**
     * Retrieves the first log entry in the queue.
     *
     * @return A pointer to the first log, or nullptr if the queue is empty.
     */
    Log *getFirst() {
        return head;
    }

    /**
     * Retrieves the last log entry in the queue.
     *
     * @return A pointer to the last log, or nullptr if the queue is empty.
     */
    Log *getLast() {
        return tail;
    }

    /**
     * Returns the current size of the queue.
     *
     * @return The number of logs in the queue.
     */
    int size() const {
        return queueSize;
    }

    /**
     * Checks if the queue is empty.
     *
     * @return True if the queue is empty, otherwise false.
     */
    bool isEmpty() const {
        return queueSize == 0;
    }

    /**
     * Clears all log entries from the queue.
     *
     * Deletes all dynamically allocated logs and resets the queue to an empty state.
     */
    void clear() {
        while (head) {
            Log *temp = head;   // Store the current head log.
            head = head->next;  // Move the head to the next log.
            delete temp;        // Delete the current log.
        }

        head = tail = nullptr;  // Reset the head and tail pointers.
        queueSize = 0;          // Reset the queue size to zero.
    }
};
