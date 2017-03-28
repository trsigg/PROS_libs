#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup

void operatorControl() {
	while (true) {
		lift.takeInput();
		drive.takeInput();
		motorSet(4, joystickGetAnalog(1, 1));
		clawL.setPower(joystickGetAnalog(1, 1));
	}
}
