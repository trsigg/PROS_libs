#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup

void operatorControl() {
	while (true) {
		lift.takeInput();
		drive.takeInput();
	}
}
