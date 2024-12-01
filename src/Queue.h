#pragma once

template <typename T>
class LogQueue {
public:
    struct Log {
        T type;
        double x;
        double y;
        Log* prev;
        Log* next;
        
        Log(const T& value, double xPos, double yPos) : type(value), x(xPos), y(yPos), prev(nullptr), next(nullptr) {}
    };

    Log* head;
    Log* tail;
    int queueSize;
    LogQueue() : head(nullptr), tail(nullptr), queueSize(0) {}

    void add(const T& value, double xPos, double yPos) {
        Log* newLog = new Log(value, xPos, yPos);
        
        if (!head) {
            head = tail = newLog;
        } else {
            tail->next = newLog;
            newLog->prev = tail;
            tail = newLog;
        }
        
        queueSize++;
    }

    Log* getFirst() {
        return head;
    }

    Log* getLast() {
        return tail;
    }

    int size() const {
        return queueSize;
    }

    bool isEmpty() const {
        return queueSize == 0;
    }

    void clear() {
        while (head) {
            Log* temp = head;
            head = head->next;
            delete temp;
        }
        head = tail = nullptr;
        queueSize = 0;
    }
};