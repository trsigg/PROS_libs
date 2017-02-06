#include "motorGroup.h"		//also includes vector and API
#include "coreIncludes.h"	//also includes cmath
#include "PID.h"
#include "Timer.h"

void MotorGroup::setPower(char power, bool overrideAbsolutes) {
	if (!overrideAbsolutes) {
		if (hasAbsMin && getPosition() <= absMin && power < -maxPowerAtAbs)
			power = -defPowerAtAbs;

		if (hasAbsMax && getPosition() >= absMax && power > maxPowerAtAbs)
			power = defPowerAtAbs;
	}

	for (unsigned char motor : motors) //set motors
		motorSet(motor, power);
}

char MotorGroup::getPower() {
	return motorGet(motors.front());	//return power of first motor in group
}

//#region constructors
MotorGroup::MotorGroup(std::vector<unsigned char> motors) : motors(motors) {
	maneuverTimer = new Timer();
}

MotorGroup::MotorGroup(std::vector<unsigned char> motors, Encoder encoder, double coeff)
												: motors(motors), encoder(encoder), encCoeff(coeff) {
	maneuverTimer = new Timer();
	encoderReset(encoder);
}

MotorGroup::MotorGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed)
												: motors(motors), potPort(potPort), potReversed(potReversed) {
	maneuverTimer = new Timer();
}
//#endregion

//#region sensors
void MotorGroup::addSensor(Encoder enc, double coeff, bool setAsDefault) {
	encoder = enc;
	encCoeff = coeff;
	encoderReset(enc);
	if (setAsDefault) potIsDefault = false;
}

void MotorGroup::addSensor(unsigned char port, bool reversed, bool setAsDefault) {
	potPort = port;
	potReversed = reversed;
	if (setAsDefault) potIsDefault = true;
}

int MotorGroup::encoderVal(bool rawValue) {
	if (hasEncoder()) {
		return encoderGet(encoder) * (rawValue ? 1 : encCoeff);
	}

	return 0;	//possible debug location
}

void MotorGroup::resetEncoder() { encoderReset(encoder); }	//possible debug location

int MotorGroup::potVal() {
	if (hasPotentiometer()) {
		return potReversed ? 4095-analogRead(potPort) : analogRead(potPort);
	} else {
		return 0;	//possible debug location
	}
}

int MotorGroup::getPosition() {
	if (hasPotentiometer() && hasEncoder()) {
		return potIsDefault ? potVal() : encoderVal();
	} else {
		return (hasEncoder() ? encoderVal() : potVal());
	}
}
//#endregion

//#region position movement
void MotorGroup::moveTowardPosition(int pos, char power) {
	return setPower(copysign(power, pos-getPosition()));
}

void MotorGroup::stopManeuver() {
	maneuverExecuting = false;
	setPower(0);
}

void MotorGroup::executeManeuver() {
	if (maneuverExecuting) {
		if (forward == (getPosition() < maneuverTarget)) {
			maneuverTimer->reset();
			setPower(maneuverPower);
		} else if (maneuverTimer->time() > maneuverTimeout) {
			maneuverExecuting = false;
			setPower(endPower);
		}
	}
}

void MotorGroup::goToPosition(int pos, bool runAsManeuver, char endPower, char maneuverPower, unsigned short timeout) {
	maneuverTarget = pos;
	endPower = endPower;
	forward = maneuverTarget > getPosition();
	maneuverPower = copysign(maneuverPower, (forward ? 1 : -1));
	maneuverExecuting = true;
	maneuverTimeout = timeout;
	maneuverTimer->reset();

	if (!runAsManeuver) {
		while (maneuverExecuting) executeManeuver();
	}
}

	//#subregion position targeting
void MotorGroup::setPosPIDconsts(double kP, double kI, double kD) {
	targetPosPID = new PID(0, kP, kI, kD);
}

void MotorGroup::setTargetPosition(int position) {
	targetPosPID->changeTarget(position);
	targetingActive = true;
}

void MotorGroup::maintainTargetPos() {
	if (targetingActive && targetPosPID) {
		setPower(targetPosPID->evaluate(getPosition()));
	}
}
	//#endsubregion
//#endregion

//#region accessors and mutators
	//#subregion sensors
bool MotorGroup::isPotReversed() { return potReversed; }
void MotorGroup::setPotReversed(bool reversed) { potReversed = reversed; }
bool MotorGroup::isPotDefault() { return potIsDefault; }
void MotorGroup::setDefaultSensor(bool potIsDefault) { this->potIsDefault = potIsDefault; }
Encoder MotorGroup::getEncoderPtr() { return encoder; }
unsigned char MotorGroup::getPotPort() { return potPort; }
bool MotorGroup::hasEncoder() { return encoder; }
bool MotorGroup::hasPotentiometer() { return potPort; }
	//#endsubregion
	//#subregion automovement
bool MotorGroup::isManeuverExecuting() { return maneuverExecuting; }
void MotorGroup::activatePositionTargeting() { targetingActive = true; }
void MotorGroup::deactivatePositionTargeting() { targetingActive = false; }
	//#endsubregion
	//#subregion position limits
void MotorGroup::setAbsMin(int min, char defPowerAtAbs, char maxPowerAtAbs) {
	absMin = min;
	hasAbsMin = true;
	this->maxPowerAtAbs = maxPowerAtAbs;
	this->defPowerAtAbs = defPowerAtAbs;
}
void MotorGroup::setAbsMax(int max, char defPowerAtAbs, char maxPowerAtAbs) {
	absMax = max;
	hasAbsMax = true;
	this->maxPowerAtAbs = maxPowerAtAbs;
	this->defPowerAtAbs = defPowerAtAbs;
}
void MotorGroup::setAbsolutes(int min, int max, char defPowerAtAbs, char maxPowerAtAbs) {
	absMin = min;
	absMax = max;
	hasAbsMin = true;
	hasAbsMax = true;
	this->maxPowerAtAbs = maxPowerAtAbs;
	this->defPowerAtAbs = defPowerAtAbs;
}
int MotorGroup::getAbsMin() { return absMin; }
int MotorGroup::getAbsMax() { return absMax; }
char MotorGroup::getDefPowerAtAbs() { return defPowerAtAbs; }
void MotorGroup::setDefPowerAtAbs(char power) { defPowerAtAbs = power; }
char MotorGroup::getMaxPowerAtAbs() { return maxPowerAtAbs; }
void MotorGroup::setMaxPowerAtAbs(char power) { maxPowerAtAbs = power; }
	//#endsubregion
//#endregion
