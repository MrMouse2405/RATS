/*
 * File: UserInterface.cpp
 *
 * Description:
 * This file implements the UserInterface namespace, which provides
 * methods for managing user interactions with the robot. It includes
 * functions to display messages, error screens, and prompts, as well
 * as manage sound feedback.
 *
 * Author: OCdt Syed & OCdt Gratton
 * Version: 2024-12-01
 */


#include "UserInterface.h"

// Display and button objects for user interaction.
Pololu3piPlus32U4::OLED display;
Pololu3piPlus32U4::ButtonB buttonB;

// Buzzer object to play sounds.
typedef Pololu3piPlus32U4::Buzzer Buzzer;

/**
 * Displays a string centered on the specified line of the display.
 *
 * This function calculates the starting position for the string based on
 * its length and the display's width, then prints the string centered on
 * the specified line.
 *
 * @param message The string to be displayed.
 * @param line The 0-based line number where the string should be centered.
 */
void displayCentered(const String &message, const uint8_t line) {
    display.gotoXY(10 - message.length() / 2, line); // Calculate center position.
    display.print(message.c_str());                 // Print the message.
}

/**
 * Plays a musical note sequence using the buzzer.
 *
 * Stops any currently playing notes before starting the new sequence.
 * If the `yield` parameter is true, this function blocks until the sequence finishes.
 *
 * @param sequence The musical note sequence to be played.
 * @param yield If true, blocks execution until the sequence finishes.
 */
void playNote(const String &sequence, const bool yield) {
    Buzzer::stopPlaying();        // Stop any previous notes.
    Buzzer::play(sequence.c_str()); // Play the new sequence.
    if (yield) {
        while (Buzzer::isPlaying()) {
            // Wait until the sequence finishes.
        }
    }
}

/**
 * Displays an error message on the screen with a sound alert.
 *
 * The message is displayed with a "[ ERROR ]" prefix and plays a
 * predefined beep sequence to alert the user. Waits for a button press
 * before clearing the display.
 *
 * @param message The error message to display.
 */
void displayError(const String &message) {
    display.clear();                       // Clear the display.
    playNote(BEEP_SEQUENCE, true);         // Play an alert sound.
    displayCentered("[ ERROR ]", 0);       // Display the error prefix.
    displayCentered(message, 1);           // Display the error message.
    buttonB.waitForButton();               // Wait for the user to press button B.
    display.clear();                       // Clear the display after acknowledgment.
}

/**
 * Initializes the user interface.
 *
 * Sets up the display with a layout of 21 characters per line across 8 lines.
 */
void UserInterface::initializeUI() {
    display.setLayout21x8();
}

/**
 * Displays the welcome screen with developer names and lab information.
 *
 * Waits for the user to press button B before clearing the screen.
 */
void UserInterface::showWelcomeScreen() {
    displayCentered("Abdul Mannan Syed", 0); // Developer name 1.
    displayCentered("Nathan Gratton", 1);   // Developer name 2.
    displayCentered("Lab 5: RATS", 4);      // Lab information.
    displayCentered("To start, press B", 7); // Instructions for starting.
    buttonB.waitForButton();                // Wait for user input.
    display.clear();                        // Clear the screen.
}

/**
 * Displays a "Ready to Go" screen.
 *
 * Shows "Ready" and "< GO >" messages. Waits for button B press before clearing.
 */
void UserInterface::showGoScreen() {
    displayCentered("Ready", 1);   // Ready message.
    displayCentered("<  GO  >", 4); // Go message.
    buttonB.waitForButton();       // Wait for user input.
    display.clear();               // Clear the screen.
}

/**
 * Displays an error message and stops the robot.
 *
 * Stops the motors and shows the error message on the screen.
 *
 * @param message The error message to display.
 */
void UserInterface::throwError(const String &message) {
    Pololu3piPlus32U4::Motors::setSpeeds(0, 0); // Stop the robot.
    displayError(message);                      // Show the error message.
}

/**
 * Displays a message on a specified line and waits for user acknowledgment.
 *
 * @param message The message to display.
 * @param line The line number where the message should appear.
 */
void UserInterface::showMessage(const String &message, const int line) {
    displayCentered(message, line); // Display the message.
    buttonB.waitForButton();        // Wait for user input.
}

/**
 * Displays a message on a specified line without blocking.
 *
 * @param message The message to display.
 * @param line The line number where the message should appear.
 */
void UserInterface::showMessageNotYielding(const String &message, const int line) {
    displayCentered(message, line); // Display the message without waiting.
}

/**
 * Displays a truncated version of a message on a specified line.
 *
 * Limits the message to 20 characters to fit within the display width.
 *
 * @param message The message to display.
 * @param line The line number where the message should appear.
 */
void UserInterface::showMessageTruncate(const String &message, const int line) {
    String truncated = message.substring(0, 20); // Truncate the message to 20 characters.
    displayCentered(truncated, line);           // Display the truncated message.
}

/**
 * Clears the display.
 */
void UserInterface::clearScreen() {
    display.clear(); // Clear all content from the display.
}
