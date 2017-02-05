#include "quadRamp.h"
#include "coreIncludes.h" //also includes math

QuadRamp::QuadRamp(float target, float initial, float maximum, float end) {
  a = (pow(target, 2) * (end+initial-2*maximum) - 2*sqrt(pow(target, 4) * (end-maximum) * (initial-maximum))) / pow(target, 4);
	b = ((end-initial)/target - a*target) * sgn(target);
  c = initial;
}

double QuadRamp::evaluate(double input) {
  return a*pow(input, 2) + b*input + c;
}
