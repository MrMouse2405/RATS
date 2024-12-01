#pragma once

template<typename T>
class Option {
public:

    Option(T value) : present(true), value(value) {}
    Option() : present (false) {}

    bool exists() {return this->present;}

    T get() {return this->value;}
    T* getPtr() {return &this->value;}

    void makePresent(T value) {
        this->present = true;
        this->value = value;
    }

    T* makeAbsent() {
        this->present = false;
        return &this->value;
    }
private:
    bool present;

    mutable T value;
};