#include "quadRamp.h"
#include "coreIncludes.h" //also includes cmath

QuadRamp::QuadRamp(float target, float initial, float maximum, float end) {
  a = ((end + initial - 2*maximum) - 2*sqrt((end-maximum) * (initial-maximum))) / pow(target, 2);
	b = ((end-initial)/target - a*target) * sgn(target);
  c = initial;
}

double QuadRamp::evaluate(double input) {
  return a*pow(input, 2) + b*input + c;
}
