#include "main.h"
#include "config.h"

void operatorControl() {
	while (true) {
		lift.takeInput();
		drive.takeInput();
	}
}
