#pragma once
template<typename T>
class Option {
public:
    Option(T value) : exist(true), value(value) {}
    Option() : exist (false) {}
    ~Option() {delete value}
    const T* get() {return &value};
    const bool exists() {return exists;}
private:
    mutable T value;
    const bool exists;
};