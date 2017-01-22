#include "motorGroup.h"		//also includes vector
#include "coreIncludes.h"	//also includes <cmath>
#include "Timer.h"
#include "API.h"

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

MotorGroup::MotorGroup(std::vector<unsigned char> motors, Encoder* encoder)
												: motors(motors), encoder(encoder) {
	maneuverTimer = new Timer();
}

MotorGroup::MotorGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed)
												: motors(motors), potPort(potPort), potReversed(potReversed) {
	maneuverTimer = new Timer();
}
//#endregion

//#region sensors
void MotorGroup::addSensor(Encoder* enc, bool setAsDefault) {
	encoder = enc;
	if (setAsDefault) potIsDefault = false;
}

void MotorGroup::addSensor(unsigned char port, bool reversed, bool setAsDefault) {
	potPort = port;
	potReversed = reversed;
	if (setAsDefault) potIsDefault = true;
}

int MotorGroup::encoderVal() {
	if (hasEncoder()) {
		return encoderGet(encoder);
	}

	return 0;	//possible debug location
}

void MotorGroup::resetEncoder() { encoderReset(encoder); }

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
		return (hasEncoder() ? encoderVal(group) : potVal(group));
	}
}
//#endregion

//#region position movement
void MotorGroup::moveTowardPosition(int pos, int power) {
	return setPower(copysign(power, position-getPosition()));
}

void MotorGroup::createManeuver(int position, char endPower, char maneuverPower, unsigned short timeout) {
	targetPos = position;
	endPower = endPower;
	forward = targetPos > getPosition();
	maneuverPower = abs(maneuverPower) * (forward ? 1 : -1);
	maneuverExecuting = true;
	maneuverTimeout = timeout;
	maneuverTimer->reset();

	setPower(maneuverPower);
}

void MotorGroup::stopManeuver() {
	maneuverExecuting = false;
	setPower(0);
}

void MotorGroup::executeManeuver() {
	if (maneuverExecuting) {
		if (forward == (getPosition() < targetPos)) {
			maneuverTimer->reset();
			setPower(maneuverPower);
		} else if (maneuverTimer->time() > maneuverTimeout) {
			maneuverExecuting = false;
			setPower(endPower);
		}
	}
}

void MotorGroup::goToPosition(int pos, char endPower, char maneuverPower, unsigned short timeout) {
	Timer posTimer = new Timer();
	char displacementSign = sgn(pos - getPosition(group));
	setPower(group, displacementSign*maneuverPower);

	while (posTimer.time() < timeout) {
		if (sgn(pos - getPosition()) == displacementSign) posTimer.reset();
	}

	setPower(group, endPower);
}
//#endregion

//#region accessors and mutators
	//#subregion sensors
bool MotorGroup::isPotReversed() { return potReversed; }
void MotorGroup::setPotReversed(bool reversed) { potReversed = reversed; }
bool MotorGroup::isPotDefault() { return potIsDefault; }
void MotorGroup::setDefaultSensor(bool potIsDefault) { this->potIsDefault = potIsDefault; }
Encoder* MotorGroup::getEncoderPtr() { return encoder; }
unsigned char MotorGroup::getPotPort() { return potPort; }
bool MotorGroup::hasEncoder() { return encoder; }
bool MotorGroup::hasPotentiometer() { return potPort; }
	//#endsubregion
	//#subregion automovement
bool MotorGroup::isManeuverExecuting() { return maneuverExecuting; }
	//#endsubregion
	//#subregion position limits
void MotorGroup::setAbsMin(int min, int defPowerAtAbs, int maxPowerAtAbs) {
	absMin = min;
	hasAbsMin = true;
	this->maxPowerAtAbs = maxPowerAtAbs;
	this->defPowerAtAbs = defPowerAtAbs;
}
void MotorGroup::setAbsMax(int max, int defPowerAtAbs, int maxPowerAtAbs) {
	absMax = max;
	hasAbsMax = true;
	this->maxPowerAtAbs = maxPowerAtAbs;
	this->defPowerAtAbs = defPowerAtAbs;
}
void MotorGroup::setAbsolutes(int min, int max, int defPowerAtAbs, int maxPowerAtAbs) {
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

//#region position limiting
void setAbsMax(int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	absMax = max;
	hasAbsMax = true;
	maxPowerAtAbs = maxPowerAtAbs;
	defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	absMin = min;
	absMax = max;
	hasAbsMin = true;
	hasAbsMax = true;
	maxPowerAtAbs = maxPowerAtAbs;
	defPowerAtAbs = defPowerAtAbs;
}
//#endregion
