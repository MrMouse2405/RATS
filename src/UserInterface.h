#pragma once

#include "RATS.h"

/*
 * File: UserInterface.h
 *
 * Description:
 * This header file declares the UserInterface namespace, which provides
 * methods for user interaction via the OLED display and button inputs.
 * It includes functions for displaying messages, handling errors, and
 * clearing the screen.
 *
 * Author: OCdt Syed & OCdt Gratton
 * Version: 2024-12-01
 */

namespace UserInterface {

    /**
     * Initializes the user interface.
     *
     * Configures the OLED display layout for use in the application.
     * Must be called before displaying any messages.
     */
    void initializeUI();

    /**
     * Displays the welcome screen.
     *
     * Shows the names of developers and the lab information,
     * prompting the user to press button B to proceed.
     */
    void showWelcomeScreen();

    /**
     * Displays the "Ready to Go" screen.
     *
     * Shows a message indicating the robot is ready and prompts the user
     * to press button B to start the operation.
     */
    void showGoScreen();

    /**
     * Displays an error message and stops the robot.
     *
     * The robot's motors are stopped, and the error message is shown on
     * the OLED display. Waits for user acknowledgment before clearing the screen.
     *
     * @param message The error message to display. Should describe the nature of the issue.
     */
    void throwError(const String &message);

    /**
     * Displays a message on a specific line of the screen.
     *
     * The function centers the provided message on the specified line
     * and waits for user acknowledgment via button B.
     *
     * @param message The message to display.
     * @param line The line number (0-based) where the message should appear.
     */
    void showMessage(const String &message, const int line);

    /**
     * Displays a message on a specific line of the screen without waiting.
     *
     * The function centers the provided message on the specified line
     * but does not block execution or wait for user acknowledgment.
     *
     * @param message The message to display.
     * @param line The line number (0-based) where the message should appear.
     */
    void showMessageNotYielding(const String &message, const int line);

    /**
     * Displays a truncated version of a message on a specific line.
     *
     * If the message exceeds 20 characters, it is truncated to fit within
     * the OLED display width. The truncated message is then centered on
     * the specified line.
     *
     * @param message The message to display.
     * @param line The line number (0-based) where the message should appear.
     */
    void showMessageTruncate(const String &message, const int line);

    /**
     * Clears the entire OLED display.
     *
     * Removes all displayed content and resets the screen to a blank state.
     */
    void clearScreen();
}
