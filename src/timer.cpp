#include "timer.h"
#include <API.h>

void Timer::reset() {
  lastReset = millis();
}

unsigned long Timer::time() {
  return millis() - lastReset;
}

Timer::Timer() {
	lastReset = millis();
}
