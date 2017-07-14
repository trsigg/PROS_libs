#include "coreIncludes.h"	//also includes cmath

int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

double convertAngle(double angle, angleType input, angleType output) {
	if (input != output) {
		if (input == DEGREES) //convert to radians
			angle *= PI / 180;
		else									//convert to degrees
			angle *= 180 / PI;
	}

	return angle;
}

char sgn(double x) {
	if (x == 0)
		return 0;
	else
		return fabs(x) / x;
}
