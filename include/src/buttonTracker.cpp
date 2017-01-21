#include "buttonTracker.h" //also includes config
#include "API.h"
#include <cmath>

bool ButtonTracker::isValidButton(unsigned char buttonGroup, unsigned char button, unsigned char joystick) {
	//possible debug location
	return (joystick<=NUM_JOYSTICKS)
						&& (5<=buttonGroup && buttonGroup<=8)
						&& (button==JOY_DOWN || button==JOY_LEFT || button==JOY_UP || button==JOY_RIGHT)
						&& ((buttonGroup==7 || buttonGroup==8) || (button==JOY_DOWN || button==JOY_UP));
}

bool* ButtonTracker::getButtonPtr(unsigned char buttonGroup, unsigned char button, unsigned char joystick) {
	if (isValidButton(joystick, buttonGroup, button))
		return pressedButtons[joystick-1][buttonGroup-5][(int)log2(button)];
		//log2 necessary because button identifiers correspond to 1, 2, 4, and 8
	else
		return nullptr;
}

bool ButtonTracker::newlyPressed(unsigned char buttonGroup, unsigned char button, unsigned char joystick) {
	bool retVal = false;	//value to be returned

	if (isValidButton(joystick, buttonGroup, button)) {
		bool newState = joystickGetDigital(joystick, buttonGroup, button);
		bool* oldStatePtr = getButtonPtr(joystick, buttonGroup, button);

		retVal = newState && !*oldStatePtr;	//(is pressed now) && (was not pressed at last update)
		*oldStatePtr = newState;
	}

	return retVal;
}
