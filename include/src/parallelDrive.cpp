#include "parallelDrive.h"  //also includes coreIncludes, cmath, and vector
#include "joystickGroup.h"

void ParallelDrive::takeInput() {
  if (arcadeInput) {
    char moveVal = coeff * joystickGetAnalog(joystick, moveAxis);
    char turnVal = coeff * joystickGetAnalog(joystick, turnAxis);

    setDrivePower(moveVal+turnVal, moveVal-turnVal);
  } else {
    leftDrive->takeInput();
    rightDrive->takeInput();
  }
}

//#region power setting
void ParallelDrive::setLeftPower(char power) { leftDrive->setPower(power); }
void ParallelDrive::setRightPower(char power) { rightDrive->setPower(power); }
void ParallelDrive::setDrivePower(char left, char right) {
  leftDrive->setPower(left);
  rightDrive->setPower(right);
}
//#endregion

//#region input configuration
void ParallelDrive::configureTankInput(double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char leftAxis=3, unsigned char rightAxis=2, unsigned char joystick) {
  leftDrive->configureInput(leftAxis, coeff, powMap, maxAcc100ms, deadband, joystick);
  rightDrive->configureInput(rightAxis, coeff, powMap, maxAcc100ms, deadband, joystick);
  arcadeInput = false;
}

void ParallelDrive::configureArcadeinput(unsigned char movementAxis, unsigned char turningAxis, double coeff) {
  moveAxis = movementAxis;
  turnAxis = turningAxis;
  this->coeff = coeff;
  arcadeInput = true;
}
//#endregion

//#region constructors
ParallelDrive::ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors) {
  leftDrive = new JoystickGroup(leftMotors);
  rightDrive = new JoystickGroup(rightMotors);
}

ParallelDrive::ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, Encoder* leftEnc, Encoder* rightEnc, double wheelDiameter, double gearRatio) {
  double coeff = PI * wheelDiameter * gearRatio / 360;

  leftDrive = new JoystickGroup(leftMotors, leftEnc, coeff);
  rightDrive = new JoystickGroup(rightMotors, rightEnc, coeff);

  updateEncConfig();
}


//#endregion
