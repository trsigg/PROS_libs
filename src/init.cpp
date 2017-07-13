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
  drive.addSensor(LEFT_ENC, LEFT, WHEEL_DIAMETER);
  drive.addSensor(RIGHT_ENC, RIGHT);

  lift.addSensor(LIFT_POT);
  //#endregion
}
