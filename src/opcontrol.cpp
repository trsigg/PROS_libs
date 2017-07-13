#include "main.h"
#include "config.h"	//also includes parallelDrive and buttonGroup

void operatorControl() {
	while (true) {
		flapper.takeInput();
		drive.takeInput();
	}
}
