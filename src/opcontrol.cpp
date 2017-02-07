#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup

void operatorControl() {
	while (true) {
		unsigned char temp[] = clawL_mps;
		lift.takeInput();
		drive.takeInput();
		motorSet(4, joystickGetAnalog(1, 1));
		motorSet(temp[0], joystickGetAnalog(1, 4));
	}
}
