#include "main.h"
#include "config.h"

void initializeIO() {}

void initialize() {
  //instantiate and configure drive
  std::vector<unsigned char> leftMotors = ld_mps;
  std::vector<unsigned char> rightMotors = rd_mps;
  ParallelDrive drive(leftMotors, rightMotors);
  drive.addSensor(gyroInit(hyro, DEF_GYRO_MULTIPLIER));
  drive.addSensor(encoderInit(leftEnc), LEFT);
  drive.addSensor(encoderInit(rightEnc), RIGHT, WHEEL_DIAMETER);

  //instantiate and configure lift
  std::vector<unsigned char> liftMotors = lift_mps;
  ButtonGroup lift(liftGroup, liftMotors, -10);
  lift.addSensor(liftPot);

  //instantiate and configure claw
  std::vector<unsigned char> clawLmotors = clawL_mps;
  std::vector<unsigned char> clawRmotors = clawR_mps;
  MotorGroup clawL(clawLmotors);
  MotorGroup clawR(clawRmotors);
  clawL.addSensor(clawPotL);
  clawR.addSensor(clawPotR);
}
