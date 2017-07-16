#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup


void operatorControl() {
	while (true) {
		flapper.takeInput();
		drive.takeInput();

		if (joystickGetDigital(1, 7, JOY_UP)) {
			printf("%f\n", drive.encoderVal());
			print("Driving...\n");
			drive.drive(20);	//100
			print("Finished.\n");
		}

		if (joystickGetDigital(1, 7, JOY_DOWN)) {
			printf("%f\n\n", drive.gyroVal());
			print("Turning...\n");
			drive.turn(90);
			print("Finished.\n");
		}
	}
}
