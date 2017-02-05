#include "main.h"
#include "config.h"

void initializeIO() {}

void initialize() {
  //#region sensor setup
  drive.addSensor(gyroInit(hyro, DEF_GYRO_MULTIPLIER));
  drive.addSensor(encoderInit(leftEnc), LEFT);
  drive.addSensor(encoderInit(rightEnc), RIGHT, WHEEL_DIAMETER);

  lift.addSensor(liftPot);

  clawL.addSensor(clawPotL);
  clawR.addSensor(clawPotR);
  //#endregion
}
