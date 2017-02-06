#include "joystickGroup.h"
#include "timer.h"
#include <cmath>
#include <API.h>

char JoystickGroup::takeInput() {
	char power = 0;

  if (active) {
  	char input = joystickGetAnalog(joystick, axis);
  	power = coeff * copysign(127 * pow(input / 127.0, powMap), input);

  	if (abs(power) < deadband) power = 0;

  	//handle ramping
  	if (ramping) { //TODO: create better ramping scheme?
  		unsigned long now = millis();
  		int elapsed = now - lastUpdated;
  		char currentPower = getPower();

  		if (elapsed > msPerPowerChange) {
  			char maxDiff = elapsed / msPerPowerChange;

  			if (abs(currentPower - power) < maxDiff) {
  				lastUpdated = now;
  			} else {
          power = (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
  			  lastUpdated = now - (elapsed % msPerPowerChange);
  			}
  		}
  	}
  }

  setPower(power);
	return power;
}

void JoystickGroup::configureInput(unsigned char axis, double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char joystick) {
  active = true;

  this->joystick = joystick;
  this->axis = axis;
  this->coeff = coeff;
  this->powMap = powMap;
  this->deadband = deadband;

  if (maxAcc100ms == 0) {
    ramping = false;
  } else {
    ramping = true;
    msPerPowerChange = 100 / maxAcc100ms;
    lastUpdated = millis();
  }
}

//#region constructors
JoystickGroup::JoystickGroup(std::vector<unsigned char> motors) : MotorGroup(motors) {
  active = false;
}

JoystickGroup::JoystickGroup(unsigned char axis, std::vector<unsigned char> motors, double coeff, double powMap, unsigned char maxAcc100ms, unsigned char deadband, unsigned char joystick) : MotorGroup(motors) {
  configureInput(axis, coeff, powMap, maxAcc100ms, deadband, joystick);
}

JoystickGroup::JoystickGroup(std::vector<unsigned char> motors, Encoder encoder, double coeff) : MotorGroup(motors, encoder, coeff) {
  active = false;
}

JoystickGroup::JoystickGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed) : MotorGroup(motors, potPort, potReversed) {
  active = false;
}
//#endregion

//#region accessors and mutators
  //#subregion state
bool JoystickGroup::isActive() { return active; }
void JoystickGroup::activate() { active = true; }
void JoystickGroup::deactivate() { active = false; }
  //#endsubregion
  //#subregion ramping
bool JoystickGroup::isRamped() { return ramping; }

void JoystickGroup::startRamping() {
  ramping = true;

  if (msPerPowerChange == 0) {
    msPerPowerChange = 10;
  }

  lastUpdated = millis();
}

void JoystickGroup::startRamping(unsigned char maxAcc100ms) {
  ramping = true;
  msPerPowerChange = 100 / maxAcc100ms;
  lastUpdated = millis();
}

void JoystickGroup::stopRamping() { ramping = false; }
  //#endsubregion
//#endregion
