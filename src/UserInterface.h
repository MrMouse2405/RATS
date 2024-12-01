#include "RATS.h"

namespace UserInterface {

    void initializeUI();

    void showWelcomeScreen();

    void showGoScreen();

    void throwError(const String& message);

    void showMessage(const String & message, const int line);
    
    void showMessageNotYielding(const String & message, const int line);

    void showMessageTruncate(const String & message, const int line);
    
    void clearScreen();
}