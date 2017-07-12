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
  //drive.addSensor(gyroInit(hyro, DEF_GYRO_MULTIPLIER));
  //drive.addSensor(encoderInit(leftEnc), LEFT);
  //drive.addSensor(encoderInit(rightEnc), RIGHT, WHEEL_DIAMETER);

  lift.addSensor(liftPot);
  //#endregion
}
