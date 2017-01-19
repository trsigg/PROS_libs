#include "motorGroup.h"	//also includes vector
#include "Timer.h"
#include "API.h"

//#region constructors
MotorGroup::MotorGroup(std::vector<unsigned char> motors) : motors(motors) {
	maneuverTimer = new Timer();
}

MotorGroup::MotorGroup(std::vector<unsigned char> motors, Encoder* encoder) : motors(motors), encoder(encoder) {
	maneuverTimer = new Timer();
}

MotorGroup::MotorGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed=false) : motors(motors), potPort(potPort), potReversed(potReversed) {
	maneuverTimer = new Timer();
}
//#endregion

//#region sensors
void MotorGroup::addSensor(Encoder* enc, bool setAsDefault) {
	encoder = enc;
	if (setAsDefault) potIsDefault = false;
}

void MotorGroup::addSensor(unsigned short port, bool reversed, bool setAsDefault) {
	potPort = port;
	potReversed = reversed;
	if (setAsDefault) potIsDefault = true;
}

int MotorGroup::encoderVal() {
	if (hasEncoder()) {
		return encoderGet(encoder);
	} else {
		return 0;	//possible debug location
	}
}

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

void MotorGroup::resetEncoder() { encoderReset(encoder); }
//#endregion

//#region position limiting
void setAbsMax(int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMax = max;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsMin(motorGroup *group, int min, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->hasAbsMin = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->hasAbsMin = true;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region motor targets
void createTarget(motorGroup *group, int position, TVexJoysticks btn, int power=127, int timeout=10) {
	for (int i=0; i<numTargets; i++) {
		if (group->targets[i] == -1) {
			group->targets[i] = position;
			group->targetButtons[i] = btn;
			break;
		}
	}

	group->targetPower = power;
	group->targetTimeout = timeout;
}
//#endregion

int setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {
	if (!overrideAbsolutes) {
		if (group->hasAbsMin && getPosition(group) <= group->absMin && power < -group->maxPowerAtAbs)
			power = -group->defPowerAtAbs;

		if (group->hasAbsMax && getPosition(group) >= group->absMax && power > group->maxPowerAtAbs)
			power = group->defPowerAtAbs;
	}

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

	return power;
}

//#region position movement
int moveTowardPosition(motorGroup *group, int position, int power=127) {
	return setPower(group, power * sgn(position - getPosition(group)));
}

void executeManeuver(motorGroup *group) {
	if (group->maneuverExecuting) {
		if (group->forward == (getPosition(group) < group->targetPos)) {
			group->maneuverTimer = resetTimer();
			setPower(group, group->maneuverPower);
		}

		if (time(group->maneuverTimer) > group->maneuverTimeout) {
			group->maneuverExecuting = false;
			setPower(group, group->endPower);
		}
	}
}

void createManeuver(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=10) {
	group->targetPos = position;
	group->endPower = endPower;
	group->forward = group->targetPos > getPosition(group);
	group->maneuverPower = abs(maneuverPower) * (group->forward ? 1 : -1);
	group->maneuverExecuting = true;
	group->maneuverTimeout = timeout;
	group->maneuverTimer = resetTimer();

	setPower(group, maneuverPower);
}

void goToPosition(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=100) {
	long posTimer = resetTimer();
	int displacementSign = sgn(position - getPosition(group));
	setPower(group, displacementSign*maneuverPower);

	while (time(posTimer) < timeout) {
		if (sgn(position - getPosition(group)) == displacementSign) posTimer = resetTimer();
	}

	setPower(group, endPower);
}
//#endregion

//#region user input
void getTargetInput(motorGroup *group) {
	for (int i=0; i<numTargets; i++) {
		if (group->targets[i] == -1) {
			break;
		} else if (vexRT[group->targetButtons[i]] == 1) {
			createManeuver(group, group->targets[i], group->stillSpeed, group->targetPower, group->targetTimeout);
		}
	}
}

int handleButtonInput(motorGroup *group) {
	if (vexRT[group->posInput] == 1) {
		group->maneuverExecuting = false;
		return group->upPower;
	} else if (vexRT[group->negInput] == 1) {
		group->maneuverExecuting = false;
		return group->downPower;
	} else {
		getTargetInput(group);

		executeManeuver(group);

		if (group->maneuverExecuting)
			return group->maneuverPower;
		else
			return group->stillSpeed;
	}
}

int handleJoystickInput(motorGroup *group) {
	int power = 0;

	int input = vexRT[group->posInput];
	power = sgn(input) * group->coeff * abs(pow(input, group->powMap)) / pow(127, group->powMap-1);

	if (abs(power) < group->deadband) power = 0;

	//handle ramping
	if (group->isRamped) {
		long now = nPgmTime;
		int elapsed = now - group->lastUpdated;
		int currentPower = motor[ group->motors[0] ];

		if (elapsed > group->msPerPowerChange) {
			int maxDiff = elapsed / group->msPerPowerChange;

			if (abs(currentPower - power) < maxDiff) {
				group->lastUpdated = now;
			} else {
				power = (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				group->lastUpdated = now - (elapsed % group->msPerPowerChange);
			}
		}
	}

	return power;
}

int takeInput(motorGroup *group, bool setMotors=true) {
	int power = 0;

	switch (group->controlType) {
		case BUTTON:
			power = handleButtonInput(group);
			break;
		case JOYSTICK:
			power = handleJoystickInput(group);
			break;
	}

	if (setMotors) setPower(group, power);

	return power;
}
//#endregion
