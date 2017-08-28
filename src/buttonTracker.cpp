#include "buttonTracker.h" //also includes config
#include <API.h>
#include <cmath>

bool ButtonTracker::pressedButtons[NUM_JOYSTICKS][4][4];

bool ButtonTracker::isValidButton(unsigned char buttonGroup, unsigned char button, unsigned char joystick) {
	//possible debug location
	return (joystick<=NUM_JOYSTICKS)
						&& (5<=buttonGroup && buttonGroup<=8)
						&& (button==JOY_DOWN || button==JOY_LEFT || button==JOY_UP || button==JOY_RIGHT)
						&& ((buttonGroup==7 || buttonGroup==8) || (button==JOY_DOWN || button==JOY_UP));
}

bool ButtonTracker::newlyPressed(unsigned char buttonGroup, unsigned char button, unsigned char joystick) {
	if (isValidButton(buttonGroup, button, joystick)) {
		bool newState = joystickGetDigital(joystick, buttonGroup, button);
		bool oldState = pressedButtons[joystick-1][buttonGroup-5][(int) log2(button)];
		pressedButtons[joystick-1][buttonGroup-5][(int) log2(button)] = newState;

		return newState && !oldState;	//(is pressed now) && (was not pressed at last update)
	}

	return false;
}
