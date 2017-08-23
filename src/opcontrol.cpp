#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup
#include "buttonTracker.h"


void operatorControl() {
	int prevPos = 2000;

	while (true) {
		//flapper.takeInput();
		flapper.maintainTargetPos();
		drive.takeInput();

		if (ButtonTracker::newlyPressed(7, JOY_UP)) {
			prevPos = flapper.getPosition();
			printf("%d", prevPos);
		}

		if (joystickGetDigital(1, 7, JOY_DOWN)) {
			flapper.setTargetPosition(prevPos);
		}
	}
}

/*
printf("%f\n", drive.encoderVal());
print("Driving...\n");
drive.drive(20);	//100
print("Finished.\n");

printf("%f\n\n", drive.gyroVal());
print("Turning...\n");
drive.turn(90);
print("Finished.\n");
*/
