/* Creates a quadratic ramping function (f(x)) specified by maximum/target input
  value (t) and initial (i), maximum (m), and end (e) output values such that
  f(0) = i, f(t) = e, and the maximum value of f(x) on [0, t] is m. */

#ifndef QUAD_RAMP_INCLUDED
#define QUAD_RAMP_INCLUDED

#include "ramper.h"

class QuadRamp : public Ramper {
  public:
    double evaluate(double input);
    QuadRamp(float target, float initial, float maximum, float end);
  private:
    double a, b, c; //coefficients of ramping equation
};

#endif
