#include "PID.h"
#include "timer.h"
#include <cmath>

double PID::evaluate(double input) {
	unsigned long elapsed = updateTimer->time();

	if (elapsed > minSampleTime) {
		updateTimer->reset();
		double error = target - input;
		unsigned short timeCorrectionFactor = useTimeAdjustment ? elapsed : 1;

		integral += (integralMax!=0 && fabs(error)>integralMax) ? error : copysign(integralMax, error);
		//adds error if |error| < integralMax, otherwise add integralMax*sgn(error)

		prevOutput = kP*error + kI*integral*timeCorrectionFactor + kD*(error - prevError)/timeCorrectionFactor;
		prevError = error;
	}

	return prevOutput;
}

void PID::reset() {
	integral = 0;
	prevError = 0;
	updateTimer->reset();
}

void PID::changeTarget(double target) {
	this->target = target;
	reset();
}

PID::PID(double target, double kP, double kI, double kD, unsigned short minSampleTime, double integralMax, bool useTimeAdjustment)
					: target(target), kP(kP), kI(kI), kD(kD), minSampleTime(minSampleTime), integralMax(fabs(integralMax)), useTimeAdjustment(useTimeAdjustment) {
	integral = 0;
	prevError = 0;
	updateTimer = new Timer();
}

//#region accessors and mutators
double PID::get_kP() { return kP; }
double PID::get_kI() { return kI; }
double PID::get_kD() { return kD; }
void PID::setCoeffs(double kP, double kI, double kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
}
double PID::getTarget() { return target; }
double PID::getIntegral() { return integral; }
void PID::setIntegral(double integral) { this->integral = integral; }
unsigned short PID::getMinSampleTime() { return minSampleTime; }
void PID::setMinSampleTime(unsigned short minSampleTime) { this->minSampleTime = minSampleTime; }
double PID::getIntegralMax() { return integralMax; }
void PID::setIntegralMax(double max) { this->integralMax = integralMax; }
//#endregion
