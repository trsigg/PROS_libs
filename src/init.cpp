#include "main.h"
#include "config.h"

extern "C" {
  void __libc_init_array();
}

void initializeIO() {
  __libc_init_array();
}

void initialize() {
  //#region sensor config
  drive.addSensor(HYRO);
  drive.addSensor(LEFT_ENC, LEFT, WHEEL_DIAMETER);
  drive.addSensor(RIGHT_ENC, RIGHT);

  flapper.addSensor(FLAPPER_POT, true);
  //#endregion
  //#region PID config
  flapper.posPIDinit(0.2, 0.001, 0.03);
  //#endregion
}
