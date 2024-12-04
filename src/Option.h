/*
 * File: Option.h
 *
 * Description:
 * This file defines a generic `Option` class, a lightweight wrapper
 * used to handle the presence or absence of a value. It provides methods
 * for checking if a value exists, retrieving the value, and modifying the
 * presence state.
 *
 * Author: OCdt Syed
 * Version: 2024-12-01
 */

#pragma once

/**
 * A generic template class for handling optional values.
 *
 * @tparam T The type of value stored in the `Option`.
 */
template<typename T>
class Option {
public:

    /**
     * Constructor to initialize the `Option` with a value.
     *
     * @param value The value to store in the `Option`.
     */
    Option(T value) : present(true), value(value) {}

    /**
     * Default constructor to initialize an absent `Option`.
     */
    Option() : present(false) {}

    /**
     * Checks if the `Option` contains a value.
     *
     * @return True if a value exists, otherwise false.
     */
    bool exists() { return this->present; }

    /**
     * Retrieves the stored value.
     *
     * @return The value stored in the `Option`.
     */
    T get() { return this->value; }

    /**
     * Retrieves a pointer to the stored value.
     *
     * @return A pointer to the value stored in the `Option`.
     */
    T *getPtr() { return &this->value; }

    /**
     * Updates the `Option` to contain a new value and sets it as present.
     *
     * @param value The new value to store in the `Option`.
     */
    void makePresent(T value) {
        this->present = true;
        this->value = value;
    }

    /**
     * Marks the `Option` as absent and returns a pointer to the current value.
     *
     * @return A pointer to the current value.
     */
    T *makeAbsent() {
        this->present = false;
        return &this->value;
    }

private:
    bool present;        // Indicates if the `Option` contains a value.
    mutable T value;     // The stored value, mutable for flexibility.
};
