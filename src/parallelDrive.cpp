#include "parallelDrive.h"  //also includes coreIncludes, cmath, vector, and API
#include "joystickGroup.h"
#include "PID.h"
#include "quadRamp.h"
#include "timer.h"

DriveDefaults dDefs;
TurnDefaults tDefs;

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
ParallelDrive::ParallelDrive(unsigned char numMotorsL, unsigned char numMotorsR, unsigned char leftMotors[], unsigned char rightMotors[], unsigned char lEncPort1, unsigned char lEncPort2, bool lReversed, unsigned char rEncPort1, unsigned char rEncPort2, bool rReversed, double wheelDiameter, double gearRatio)
                              : wheelDiameter(wheelDiameter) {
  double coeff = PI * wheelDiameter * gearRatio / 360.0;

  leftDrive = new JoystickGroup(numMotorsL, leftMotors, lEncPort1, lEncPort2, coeff * (lReversed ? -1.0 : 1.0));
  rightDrive = new JoystickGroup(numMotorsR, rightMotors, rEncPort1, rEncPort2, coeff * (rReversed ? -1.0 : 1.0));
  updateEncConfig();

  positionTimer = new Timer;
  sampleTimer = new Timer;
  moveTimer = new Timer;

  initializeDefaults();
}

ParallelDrive::ParallelDrive(unsigned char numMotorsL, unsigned char numMotorsR, unsigned char leftMotors[], unsigned char rightMotors[], double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char leftAxis, unsigned char rightAxis, unsigned char joystick) {
  leftDrive = new JoystickGroup(numMotorsL, leftMotors);
  rightDrive = new JoystickGroup(numMotorsR, rightMotors);

  configureTankInput(coeff, powMap, maxAcc100ms, deadband, leftAxis, rightAxis, joystick);

  positionTimer = new Timer;
  sampleTimer = new Timer;
  moveTimer = new Timer;

  initializeDefaults();
}

ParallelDrive::ParallelDrive(unsigned char movementAxis, unsigned char turningAxis, unsigned char numMotorsL, unsigned char numMotorsR, unsigned char leftMotors[], unsigned char rightMotors[], double coeff) {
  leftDrive = new JoystickGroup(numMotorsL, leftMotors);
  rightDrive = new JoystickGroup(numMotorsR, rightMotors);

  configureArcadeInput(movementAxis, turningAxis, coeff);

  positionTimer = new Timer;
  sampleTimer = new Timer;
  moveTimer = new Timer;

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
void ParallelDrive::addSensor(unsigned char encPort1, unsigned char encPort2, bool reversed, encoderConfig side, double wheelDiameter, double gearRatio) {
  if (wheelDiameter == 0) {
    if (this->wheelDiameter != 0) {
      wheelDiameter = this->wheelDiameter;
    } else {
      wheelDiameter = 3.25; //possible debug location
    }
  } else {
    this->wheelDiameter = wheelDiameter;
  }

  double coeff = PI * wheelDiameter * gearRatio / 360.0 * (reversed ? -1.0 : 1.0);

  if (side == LEFT) {
    leftDrive->addSensor(encPort1, encPort2, coeff);
  } else if (side == RIGHT) {
    rightDrive->addSensor(encPort1, encPort2, coeff);
  } else if (leftDrive->hasEncoder()) {
    rightDrive->addSensor(encPort1, encPort2, coeff);
  } else {
    leftDrive->addSensor(encPort1, encPort2, coeff); //possible debug location (if both encoders are attached)
  }

  updateEncConfig();
}

void ParallelDrive::addSensor(unsigned char gyroPort, gyroCorrectionType correction, unsigned short multiplier) {
  gyro = gyroInit(gyroPort, multiplier);
  gyroCorrection = correction;
}

double ParallelDrive::encoderVal(encoderConfig side, bool rawValue, bool absolute) {
  if (side == UNASSIGNED)
		side = encConfig;

	if (side == AVERAGE) {
		if (absolute) {
			return (fabs(encoderVal(LEFT, rawValue)) + fabs(encoderVal(RIGHT, rawValue))) / 2;
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

				totalWidth += fabs(encoderVal() / gyroVal(RADIANS));
				samples++;
			}
		}
		width = totalWidth / samples;
	}

  return width;
}
//#endregion

//#region automovement
void ParallelDrive::turn(double angle, bool runAsManeuver, double rc1, double rc2, double rc3, double rc4, double rc5, angleType format, unsigned short waitAtEnd, unsigned short sampleTime, char brakePower, unsigned short brakeDuration, bool useGyro) {
  //initialize variables
  if (reverseTurns) angle *= -1;
	double formattedAngle = convertAngle(angle, format, DEGREES);
	target = (useGyro ? formattedAngle : formattedAngle*PI*width/180.0/wheelDiameter); //possible debug location (if not all variables are initialized)
	finalDelay = waitAtEnd;
  this->sampleTime = sampleTime;
	brakeDelay = brakeDuration;
	usingGyro = useGyro;
	isTurning = true;

  if (rc4 == 0) {
    ramp = new QuadRamp(target, rc1, rc2, rc3);
    quadRamping = true;
  } else {
    ramp = new PID(target, rc1, rc5, rc2);
    margin = convertAngle(rc3, format, DEGREES);
    timeout = rc4;
    quadRamping = false;
    maneuverTimer->reset();
  }

	resetGyro();

	if (!runAsManeuver) {
		while (isTurning)
			executeManeuver();
	}
}

void ParallelDrive::drive(double dist, bool runAsManeuver, double rc1, double rc2, double rc3, double rc4, double rc5, unsigned short waitAtEnd, double kP, double kI, double kD, correctionType correction, bool rawValue, double minSpeed, unsigned short moveTimeout, char brakePower, unsigned short brakeDuration, unsigned short sampleTime) {
  //initialize variables
	target = dist;
	this->rawValue = rawValue;
	minSpeed = minSpeed * sampleTime / 1000;
	this->moveTimeout = moveTimeout;
  brakeDelay = limit(0, brakeDuration, waitAtEnd);
	finalDelay = waitAtEnd - brakeDuration;
	this->sampleTime = sampleTime;
	isDriving = true;
	correctionPID = new PID(0, kP, kI, kD);

	leftDist = 0;
	rightDist = 0;
	totalDist = 0;

  if (rc4 == 0) {
    ramp = new QuadRamp(target, rc1, rc2, rc3);
    quadRamping = true;
  } else {
    ramp = new PID(target, rc1, rc5, rc2);
    margin = rc3;
    timeout = rc4;
    quadRamping = false;
    maneuverTimer->reset();
  }

  if (correction == NONE)
    setCorrectionType(NONE);
	else if (correction == GYRO)
		setCorrectionType(GYRO);
	else //correction is UNASSIGNED or ENCODER
		setCorrectionType(ENCODER);

	//initialize sensors
	resetEncoders();
	sampleTimer->reset();
  moveTimer->reset();

  if (!runAsManeuver) {
    while (isDriving)
      executeManeuver();
  }
}

void ParallelDrive::executeManeuver() { //TODO: break up into smaller functions
  if (isDriving && sampleTimer->time() >= sampleTime) {  //driving
    if (moveTimer->time() >= moveTimeout) {  //timed out due to lack of movement
      setDrivePower(0, 0);
      isDriving = false;
    }
    else if (!maneuverFinished()) {  //continue driving
      //update distances
      leftDist += fabs(encoderVal(LEFT, rawValue));
  	  rightDist += fabs(encoderVal(RIGHT, rawValue));
  	  totalDist = (leftDist + rightDist) / 2;

      //update timers
      sampleTimer->reset();
      if (encoderVal() >= minSpeed) moveTimer->reset();
      if (!quadRamping && fabs(totalDist - target) > margin) maneuverTimer->reset();

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
    }
    else {  //end maneuver - TAKES TIME! Expect a delay unless this is wrapped in a task
      if (quadRamping) {  //brake
      	setDrivePower(-sgn(target)*brakePower, -sgn(target)*brakePower);
      	delay(brakeDelay);
      }

      setDrivePower(0, 0);
    	delay(finalDelay);
    	isDriving = false;
    }
  }
  else if (isTurning) { //turning
    if (!maneuverFinished()) {
      double progress = maneuverProgress();

      if (!quadRamping && fabs(progress - target) > margin) //track timeout state
        maneuverTimer->reset();

      char power = ramp->evaluate(progress);

      setDrivePower(sgn(target)*power, -sgn(target)*power);
    }
    else {  //end maneuver - TAKES TIME! Expect a delay unless this is wrapped in a task
      if (quadRamping) {  //brake
      	setDrivePower(-sgn(target)*brakePower, sgn(target)*brakePower);
      	delay(brakeDelay);
      }

      setDrivePower(0, 0);
    	delay(finalDelay);
    	isTurning = false;
    }
  }
}

double ParallelDrive::maneuverProgress(angleType format) {
  if (isDriving) {
    return totalDist;
  } else if (isTurning) {
    if (usingGyro)
      return fabs(gyroVal(format));
    else
      return convertAngle(encoderVal()*PI*width/180.0/wheelDiameter, DEGREES, format);
  }

  return 0;
}

bool ParallelDrive::maneuverFinished() {
  return (quadRamping && maneuverProgress() >= fabs(target))
          || (!quadRamping && maneuverTimer->time() >= timeout);
}

bool ParallelDrive::maneuverExecuting() {
  return isDriving || isTurning;
}

void ParallelDrive::initializeDefaults() {
  //turning
  tDefs.defAngleType = DEGREES;
  tDefs.useGyro = true;
  tDefs.brakePower = 15;
  tDefs.waitAtEnd = 100;
  tDefs.sampleTime = 30;
  tDefs.brakeDuration = 100;
  tDefs.rampConst1 = 40;		// initialPower/kP
	tDefs.rampConst2 = 127;		// maxPower/kD
	tDefs.rampConst3 = -30;		// finalPower/error
	tDefs.rampConst4 = 0;			// 0/pd timeout
	tDefs.rampConst5 = 0.005;	// irrelevant/kI

  //driving
  dDefs.defCorrectionType = AUTO;
  dDefs.rawValue = false;
  dDefs.brakePower = 30;
  dDefs.waitAtEnd = 100;
  dDefs.sampleTime = 50;
  dDefs.brakeDuration = 100;
  dDefs.moveTimeout = 1000;
  dDefs.rampConst1 = 50;	//same as above
	dDefs.rampConst2 = 120;
	dDefs.rampConst3 = -20;
	dDefs.rampConst4 = 0;
	dDefs.rampConst5 = 0.05;
  dDefs.kP_c = 0.55;
  dDefs.kI_c = 0.007;
  dDefs.kD_c = 0.15;
  dDefs.minSpeed = 10;
}
//#endregion

//#region accessors and mutators
  //#subregion sensors
void ParallelDrive::setEncoderConfig(encoderConfig config) { encConfig = config; }
void ParallelDrive::setAbsAngle(double angle, angleType format) {
  angle = convertAngle(angle, format, DEGREES);
  angleOffset = angle - gyroVal();
}
bool ParallelDrive::hasGyro() { return gyro; }  //TODO: verify that this works
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
		while (fabs(gyroVal()) > 1) resetGyro(); //I'm horrible, I know (this is here so gyro is only reset when correcting with it)
	} else if (type==ENCODER && leftDrive->hasEncoder() && rightDrive->hasEncoder()) {
		correction = ENCODER;
	} else {
		correction = NONE;
	}
}

void ParallelDrive::setTurnReversal(bool reversed) {  reverseTurns = reversed;  }
  //#endsubregion
//#endregion
