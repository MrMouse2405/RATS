#pragma once
#include "Option.hpp"
#include <stdlib.h>


template <typename TaskEnum, typename Task,size_t NumberOfEnums>
class TaskPriorityLinkedList {
public:
    TaskPriorityLinkedList();
    ~TaskPriorityLinkedList() {}
    void setUpTask(TaskEnum enum, Task t) {
        this->array[enum].t = t;
    }

    void reset() {
        this->head = Option<TaskEnum>();
        fired = 0;
    }

    void fire(TaskEnum enum) {
        if (head.exists()) {
            array[zip].next = enum;
            zip = enum;
        }
        else {
            head = Option<TaskEnum>(enum);
            zip = enum;
        }
        fired += 1;
    }

    void next() {

    }

    void reset
private:
    struct Item {
        Task t;
        TaskEnum next;
    }
    struct Item array[NumberOfEnums];
    Option<TaskEnum> head = Option<TaskEnum>();
    TaskEnum zip;
    size_t fired = 0;
    size_t at = 0;
};