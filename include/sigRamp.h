/* Creates a sigmoid (logistic) ramping function specified by a maximum absolute
  output value (M), a y-intercept (i), and a ramping constant (k) which
  determines the aggressiveness of ramping.
  (See desmos.com/calculator/ht1pwoxoxr for a graph) */

#ifndef SIG_RAMP_INCLUDED
#define SIG_RAMP_INCLUDED

#include "ramper.h"

class SigRamp : public Ramper {
  public:
    double evaluate(double input);
    SigRamp(double k=0.0005, double M=127, double intercept=10);
  private:
    double k, M2, i, C, s;  //tuning constants in logistic equation
};

#endif
