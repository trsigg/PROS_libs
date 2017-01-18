#include "coreIncludes.h"	//also includes <cmath>

int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

double convertAngle(double angle, angleType output, angleType input) {
	if (input != output) {
		if (input == DEGREES) { //convert input to RAW
			angle *= 10;
		} else if (input == RADIANS) {
			angle *= 1800 / PI;
		}

		if (output == DEGREES) {
			angle /= 10;
		} else if (output == RADIANS) {
			angle *= PI / 1800;
		}
	}

	return angle;
}

char sgn(double x) {
	if (x == 0)
		return 0;
	else
		return std::abs(x) / x;
}
