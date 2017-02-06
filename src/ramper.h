/* A general base class for ramping algorithms */

#ifndef RAMPER_INCLUDED
#define RAMPER_INCLUDED

class Ramper {
  public:
    virtual double evaluate(double input);
};

#endif
