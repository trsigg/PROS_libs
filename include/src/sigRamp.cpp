#include "sigRamp.h"
#include "math.h"

SigRamp::SigRamp(double k, double M, double intercept) : k(k), M2(2*M), i(intercept) {
  C = 2 * M / i - 1;
  s = log((M2/(M+i) - 1) / C) / (k * M2);
}

double SigRamp::evaluate(double input) {
  return M2 / (1 + C * exp(-k * M2 * (input + s)));
}
