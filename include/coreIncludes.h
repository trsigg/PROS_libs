/* Useful stand-alone functions, datatypes, and constants */

#ifndef COREINCLUDES_INCLUDED
#define COREINCLUDES_INCLUDED

#include <cmath>

const double PI = acos(-1);
/* Because math doesn't have a pi constant for some reason */

enum angleType { DEGREES, RADIANS, RAW };
/*Used for specifying the format of an angle.

  RAW corresponds to 10*degrees, which is how VEX gyros format angles */

int limit(int input, int min, int max);
/* Restricts input to the interval [min, max]

  If input<min, returns min;
  If input>max, returns max;
  Otherwise, returns input. */


double convertAngle(double angle, angleType output, angleType input=RAW);
/* Converts angle of type input to type output */

char sgn(double x);
/* The signum function. Returns -1 for x<0, 0 for x=0, and 1 for x>0. */

#endif
