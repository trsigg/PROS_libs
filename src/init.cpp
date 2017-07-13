#include "main.h"
#include "config.h"

extern "C" {
  void __libc_init_array();
}

void initializeIO() {
  __libc_init_array();
}

void initialize() {
  //#region sensor configuration
  drive.addSensor(HYRO);
  drive.addSensor(LEFT_ENC, LEFT, WHEEL_DIAMETER);
  drive.addSensor(RIGHT_ENC, RIGHT);

  flapper.addSensor(FLAPPER_POT);
  //#endregion
}
