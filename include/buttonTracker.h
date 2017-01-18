/* Allows for tracking of button state for toggle-type control and
  checks validity of button combinations */

#ifndef BUTTON_TRACKER_INCLUDED
#define BUTTON_TRACKER_INCLUDED

#include "config.h"

class ButtonTracker {
  public:
    static bool newlyPressed(unsigned char joystick, unsigned char buttonGroup, unsigned char button);
    /* Returns true if button has been pressed since last call (but was not pressed previously) */
    static bool isValidButton(unsigned char joystick, unsigned char buttonGroup, unsigned char button);
    /* Returns true if joystick, buttonGroup, and button form a valid button identifier

      Checks that joystick <= NUM_JOYSTICKS (defined in config.h) and that buttonGroup
      and button correspond to a real button (i.e. allows 5U and 7R but not 9U or 6R) */
  private:
    static bool* pressedButtons[NUM_JOYSTICKS][4][4];
    /* Holds the states of the buttons at last update */
    static bool* getButtonPtr(unsigned char joystick, unsigned char buttonGroup, unsigned char button);
    /* Returns a pointer to the location in pressedButtons holding the state of the specified button */
};

#endif
