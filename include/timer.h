#ifndef TIMER_INCLUDED
#define TIMER_INCLUDED

class Timer {
  public:
    void reset();         //resets time to 0
    unsigned long time(); //returns time since last reset (or created) in milliseconds
    Timer();
  private:
    unsigned long lastReset;  //system time at last reset
};

#endif
