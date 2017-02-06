#include "parallelDrive.h"  //also includes coreIncludes, cmath, vector, and API
#include "joystickGroup.h"
#include "PID.h"
#include "quadRamp.h"
#include "timer.h"

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

//#region constructors
/*remove? ParallelDrive::ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors) {
  leftDrive = new JoystickGroup(leftMotors);
  rightDrive = new JoystickGroup(rightMotors);

  initializeDefaults();
}*/

ParallelDrive::ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, Encoder leftEnc, Encoder rightEnc, double wheelDiameter, double gearRatio) {
  double coeff = PI * wheelDiameter * gearRatio / 360;

  leftDrive = new JoystickGroup(leftMotors, leftEnc, coeff);
  rightDrive = new JoystickGroup(rightMotors, rightEnc, coeff);
  updateEncConfig();

  initializeDefaults();
}

ParallelDrive::ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char leftAxis, unsigned char rightAxis, unsigned char joystick) {
  configureTankInput(coeff, powMap, maxAcc100ms, deadband, leftAxis, rightAxis, joystick);

  initializeDefaults();
}

ParallelDrive::ParallelDrive(unsigned char movementAxis, unsigned char turningAxis, std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff) {
  configureArcadeInput(movementAxis, turningAxis, coeff);

  initializeDefaults();
}
//#endregion

//#region input configuration
void ParallelDrive::configureTankInput(double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char leftAxis, unsigned char rightAxis, unsigned char joystick) {
  leftDrive->configureInput(leftAxis, coeff, powMap, maxAcc100ms, deadband, joystick);
  rightDrive->configureInput(rightAxis, coeff, powMap, maxAcc100ms, deadband, joystick);
  arcadeInput = false;
}

void ParallelDrive::configureArcadeInput(unsigned char movementAxis, unsigned char turningAxis, double coeff) {
  moveAxis = movementAxis;
  turnAxis = turningAxis;
  this->coeff = coeff;
  arcadeInput = true;
}
//#endregion

//#region sensors
void ParallelDrive::addSensor(Encoder encoder, encoderConfig side, double wheelDiameter, double gearRatio) {
  double coeff = PI * wheelDiameter * gearRatio / 360;

  if (side == LEFT) {
    leftDrive->addSensor(encoder, coeff);
  } else if (side == RIGHT) {
    rightDrive->addSensor(encoder, coeff);
  } else if (leftDrive->hasEncoder()) {
    rightDrive->addSensor(encoder, coeff);
  } else {
    leftDrive->addSensor(encoder, coeff); //possible debug location (if both encoders are attached)
  }

  updateEncConfig();
}

double ParallelDrive::encoderVal(encoderConfig side, bool rawValue, bool absolute) {
  if (side == UNASSIGNED)
		side = encConfig;

	if (side == AVERAGE) {
		if (absolute) {
			return (abs(encoderVal(LEFT, rawValue)) + abs(encoderVal(RIGHT, rawValue))) / 2;
		} else {
			return (encoderVal(LEFT, rawValue) + encoderVal(RIGHT, rawValue)) / 2;
		}
	} else if (side == LEFT) {
		return leftDrive->encoderVal(rawValue);
	} else {
		return rightDrive->encoderVal(rawValue);
	}
}

void ParallelDrive::resetEncoders() {
    leftDrive->resetEncoder();
    rightDrive->resetEncoder();
}

double ParallelDrive::gyroVal(angleType format) {
  if (hasGyro())
    return convertAngle(gyroGet(gyro), DEGREES, format);

  return 0; //possible debug location
}

void ParallelDrive::resetGyro() {
  angleOffset += gyroVal();

  if (hasGyro())  //possible debug location
    return gyroReset(gyro);
}

double ParallelDrive::absAngle(angleType format) {
  return gyroVal() + convertAngle(angleOffset, DEGREES, format);
}

void ParallelDrive::updateEncConfig() {
  if (leftDrive->hasEncoder()) {
    if (rightDrive->hasEncoder())
      encConfig = AVERAGE;
    else
      encConfig = LEFT;
  } else if (rightDrive->hasEncoder()) {
    encConfig = RIGHT;
  } else {
    encConfig = UNASSIGNED;
  }
}
//#endregion

//#region position tracking
void ParallelDrive::updatePosition() {
  if (positionTimer->time() >= minSampleTime && width != 0) {
		double leftDist = encoderVal(LEFT);
		double rightDist = encoderVal(RIGHT);
		double angle = absAngle(RADIANS);

		resetEncoders();
		positionTimer->reset();

		if (gyroCorrection == FULL && rightDist+leftDist != 0) {
			double deltaT = angle - orientation;
			double correctionFactor = (rightDist - leftDist - width*deltaT) / (rightDist + leftDist);
			leftDist *= 1 + correctionFactor;
			rightDist *= 1 - correctionFactor;
		}

		if (rightDist!=leftDist && leftDist!=0) {
			float r = width / (rightDist/leftDist - 1.0) + width/2;
			float phi = (rightDist - leftDist) / width;

			xPos += r * (sin(orientation + phi) - sin(orientation));
			yPos += r * (cos(orientation) - cos(orientation + phi));
			orientation = (gyroCorrection==NO ? orientation+phi : angle);
		} else {
			xPos += leftDist * cos(orientation);
			yPos += leftDist * sin(orientation);
		}
	}
}

double ParallelDrive::calculateWidth(unsigned short duration, unsigned short sampleTime, char power, unsigned short reverseDelay) {
  if (hasGyro() && encConfig != UNASSIGNED) {
		Timer widthTimer;
		double totalWidth = 0;
		unsigned short samples = 0;

		for (char i=1; i>=0; i--) { //turn both directions
			setDrivePower((2*i-1) * power, (1-2*i) * power); //formula to reverse direction
			delay(reverseDelay * i); //delay second time only
			widthTimer.reset();

			while (widthTimer.time() < (duration-reverseDelay)/2) {
				resetEncoders();
				resetGyro();
				delay(sampleTime);

				totalWidth += abs(encoderVal() / gyroVal(RADIANS));
				samples++;
			}
		}
		width = totalWidth / samples;
	}

  return width;
}
//#endregion

//#region automovement
void ParallelDrive::turn(double angle, bool runAsManeuver, double in1, double in2, double in3, angleType format, unsigned short waitAtEnd, unsigned short sampleTime, char brakePower, unsigned short brakeDuration, bool useGyro) {
  //initialize variables
	double formattedAngle = convertAngle(angle, format, DEGREES);
	target = (useGyro ? formattedAngle : PI*width*formattedAngle/360);
	finalDelay = waitAtEnd;
  this->sampleTime = sampleTime;
	brakeDelay = brakeDuration;
	usingGyro = useGyro;
	isTurning = true;

  if (in1 == 0)
    ramp = new PID(target, in2, 0, in3);
  else
    ramp = new QuadRamp(target, in1, in2, in3);

	resetGyro();

	if (!runAsManeuver) {
		while (maneuverProgress() < target)
			executeManeuver();
	}
}

void ParallelDrive::drive(double dist, bool runAsManeuver, double in1, double in2, double in3, unsigned short waitAtEnd, double kP, double kI, double kD, correctionType correction, bool rawValue, double minSpeed, unsigned short timeout, char brakePower, unsigned short brakeDuration, unsigned short sampleTime) {
  //initialize variables
	target = dist;
	this->rawValue = rawValue;
	minDiffPerSample = minSpeed * sampleTime / 1000;
	this->timeout = timeout;
  brakeDelay = limit(0, brakeDuration, waitAtEnd);
	finalDelay = waitAtEnd - brakeDuration;
	this->sampleTime = sampleTime;
	isDriving = true;
	correctionPID = new PID(0, kP, kI, kD);

	leftDist = 0;
	rightDist = 0;
	totalDist = 0;

  if (in1 == 0)
    ramp = new PID(target, in2, 0, in3);
  else
    ramp = new QuadRamp(target, in1, in2, in3);

  if (correction == NONE)
    setCorrectionType(NONE);
	else if (correction == GYRO)
		setCorrectionType(GYRO);
	else //correction is UNASSIGNED or ENCODER
		setCorrectionType(ENCODER);

	//initialize sensors
	resetEncoders();
	sampleTimer->reset();
  timeoutTracker->reset();

  if (!runAsManeuver) {
    while (maneuverProgress() < target)
      executeManeuver();
  }
}

void ParallelDrive::executeManeuver() {
  if (isDriving && sampleTimer->time() > sampleTime) {  //driving
    if (maneuverProgress() < abs(target)) {
      leftDist += abs(encoderVal(LEFT, rawValue));
  	  rightDist += abs(encoderVal(RIGHT, rawValue));
  	  totalDist = (leftDist + rightDist) / 2;

      if (encoderVal() > minDiffPerSample) timeoutTracker->reset(); //track timeout state

      resetEncoders();

    	//calculate error value and correction coefficient
    	double error;

    	switch (correction) {
    		case ENCODER:
    			error = rightDist - leftDist;
    			break;
    		case GYRO:
    			error = sin(gyroVal(RADIANS)) * totalDist; //not sure if this is the right approach, but...
    			break;
    		default:
    			error = 0;
      }

      int power = ramp->evaluate(totalDist);

      double correctionPercent = 1 + correctionPID->evaluate(error);
      double rightPower = power * correctionPercent;
      double leftPower = power;

      if (rightPower > 127) {
        rightPower = 127;
        leftPower = 127 / (correctionPercent);
      }

      setDrivePower(sgn(target)*leftPower, sgn(target)*rightPower);
    } else {  //end maneuver - TAKES TIME! Expect a delay unless this is wrapped in a task
      //brake
    	setDrivePower(-sgn(target)*brakePower, -sgn(target)*brakePower);
    	delay(brakeDelay);
    	setDrivePower(0, 0);

    	delay(finalDelay);
    	isDriving = false;
    }
  } else if (isTurning) { //turning
    if (maneuverProgress() < abs(target)) {
      char power = ramp->evaluate(maneuverProgress());

      setDrivePower(sgn(target)*power, -sgn(target)*power);
    } else {  //end maneuver - TAKES TIME! Expect a delay unless this is wrapped in a task
      //brake
      setDrivePower(-sgn(target)*brakePower, sgn(target)*brakePower);
      delay(brakeDelay);
      setDrivePower(0, 0);

      delay(finalDelay);
      isDriving = false;
    }
  }
}

double ParallelDrive::maneuverProgress(angleType format) {
  if (isDriving) {
    return totalDist;
  } else if (isTurning) {
    if (usingGyro)
      return gyroVal(format);
    else
      return encoderVal();
  }

  return 0;
}

bool ParallelDrive::maneuverExecuting() {
  return isDriving || isTurning;
}

void initializeDefaults() {
  //turning
  tDefs.defAngleType = DEGREES;
  tDefs.useGyro = true;
  tDefs.brakePower = 20;
  tDefs.waitAtEnd = 100;
  tDefs.sampleTime = 30;
  tDefs.brakeDuration = 100;
  tDefs.rampConst1 = 50;
  tDefs.rampConst2 = 127;
  tDefs.rampConst3 = -15;

  //driving
  dDefs.defCorrectionType = AUTO;
  dDefs.rawValue = false;
  dDefs.brakePower = 40;
  dDefs.waitAtEnd = 100;
  dDefs.sampleTime = 50;
  dDefs.brakeDuration = 100;
  dDefs.timeout = 1000;
  dDefs.rampConst1 = 40;
  dDefs.rampConst2 = 120;
  dDefs.rampConst3 = -15;
  dDefs.kP_c = 0.25;
  dDefs.kI_c = 0.25;
  dDefs.kD_c = 0.25;
  dDefs.minDiffPerSample = 5;
}
//#endregion

//#region accessors and mutators
  //#subregion sensors
void ParallelDrive::setEncoderConfig(encoderConfig config) { encConfig = config; }
void ParallelDrive::setAbsAngle(double angle, angleType format) {
  angle = convertAngle(angle, format, DEGREES);
  angleOffset = angle - gyroVal();
}
Gyro* ParallelDrive::getGyroPtr() { return gyro; }
bool ParallelDrive::hasGyro() { return gyro; }
  //#endsubregion
  //#subregion position tracking
void ParallelDrive::setWidth(double inches) { width = inches; }
void ParallelDrive::setRobotPosition(double x, double y, double theta, angleType format, bool updateAngleOffset) {
  xPos = x;
  yPos = y;
  orientation = convertAngle(theta, format, RADIANS);
  if (updateAngleOffset) setAbsAngle(convertAngle(theta, format, DEGREES));
}
double ParallelDrive::x() { return xPos; }
double ParallelDrive::y() { return yPos; }
double ParallelDrive::theta(angleType format) { return convertAngle(orientation, RADIANS, format); }
  //#endsubregion
  //#subregion autonomous
void ParallelDrive::setCorrectionType(correctionType type) {
  if (type==GYRO && hasGyro()) {
		correction = GYRO;
		while (abs(gyroVal()) > 10) resetGyro(); //I'm horrible, I know (this is here so gyro is only reset when correcting with it)
	} else if (type==ENCODER && leftDrive->hasEncoder() && rightDrive->hasEncoder()) {
		correction = ENCODER;
	} else {
		correction = NONE;
	}
}
  //#endsubregion
//#endregion
