#include "UserInterface.h"

Pololu3piPlus32U4::OLED display;
Pololu3piPlus32U4::ButtonB buttonB;

typedef Pololu3piPlus32U4::Buzzer Buzzer;

/**
 * Displays a string centered on the specified line of a display.
 *
 * This function takes a string and positions it in the center of a given line
 * on the display. The string is adjusted based on the display's width to ensure
 * that it is evenly spaced on both sides.
 *
 * @param message The string to be displayed.
 * @param line The line number on which the string should be centered (0-based).
 *             Ensure that the line number corresponds to a valid line on the display.
 */
void displayCentered(const String &message, const uint8_t line) {
    // 10 is half of 21 (see function setup)
    display.gotoXY(10 - message.length() / 2, line);
    display.print(message.c_str());
}

/**
 * Plays a specified musical note sequence.
 *
 * This function takes a string representing a note sequence and
 * plays it using the buzzer. It blocks execution until the note
 * sequence has finished playing.
 *
 * @param sequence The note sequence to be played.
 * @param yield The program will be yielded until buzzer has
 *              finished playing if set to true
 */
void playNote(const String &sequence, const bool yield) {
    Buzzer::stopPlaying(); // stop all previous
    Buzzer::play(sequence.c_str());
    if (yield) {
        while (Buzzer::isPlaying()) {
        }
    }
}


/**
 * Displays an error message on the screen.
 *
 * This function takes a string as an input and displays it as an error message.
 * It may format the message with a specific prefix or styling to indicate that
 * it is an error, ensuring that the user can easily identify it.
 *
 * @param message The error message to be displayed. It should provide clear information
 *          about the nature of the error.
 */
void displayError(const String &message) {
    display.clear();
    playNote(BEEP_SEQUENCE, true);
    displayCentered("[ ERROR ]", 0);
    displayCentered(message, 1);
    buttonB.waitForButton();
    display.clear();
}

void UserInterface::initializeUI() {
    display.setLayout21x8();
}


void UserInterface::showWelcomeScreen() {
    displayCentered("Abdul Mannan Syed", 0);
    displayCentered("Nathan Gratton", 1);
    displayCentered("Lab 5: RATS", 4);
    displayCentered("To start, press B", 7);
    buttonB.waitForButton();
    display.clear();
}

void UserInterface::showGoScreen() {
    displayCentered("Ready", 1);
    displayCentered("<  GO  >", 4);
    buttonB.waitForButton();
    display.clear();
}

void UserInterface::throwError(const String& message) {
    Pololu3piPlus32U4::Motors::setSpeeds(0,0);
    displayError(message);
}

void UserInterface::showMessage(const String& message,const int line) {
    displayCentered(message,line);
    buttonB.waitForButton();
}

void UserInterface::showMessageNotYielding(const String& message,const int line) {
    displayCentered(message,line);
 }

void UserInterface::clearScreen() {
    display.clear();
}