/* A general PID controller with adjustable coefficients, target value,
	sample time and integral restrictions */

#ifndef PID_INCLUDED
#define PID_INCLUDED

class Timer;

class PID {
	public:
		double evaluate(double input);		//performs PID calculations
		void reset();       					 		//sets integral and prev-error to zero and resets updateTimer
		void changeTarget(double target);	//sets target and calls reset()
    PID(double target, double kP, double kI, double kD, unsigned short minSampleTime=30, double integralMax=0, bool useTimeAdjustment=false);
		//accessors and mutators
		double get_kP();
    double get_kI();
    double get_kD();
		void setCoeffs(double kP, double kI, double kD);
		double getTarget();
		double getIntegral();
		void setIntegral(double integral);
		unsigned short getMinSampleTime();
		void setMinSampleTime(unsigned short time);
		double getIntegralMin();
		void setIntegralMin(double min);
		double getIntegralMax();
		void setIntegralMax(double max);

	private:
		Timer* updateTimer; //stores time since last evaluation
		double integral;		//integral of error value
		double prevError;   //error value at last evaluation
		double prevOutput;	//output at last evaluation
		//configuration (user set)
		double target;
		double kP, kI, kD;						//tuning coefficients
		unsigned short minSampleTime;	//minimum time (milliseconds) before accepting new input
		double integralMax; 					//Maximum absolute error value which will be added to the integral (inactive if 0)
		bool useTimeAdjustment;				//whether to adjust integral and derivative calculation by time interval between evaluations
};

#endif
