/* General class for modeling a single block of motors as a group

  Supports setting motors, associating encoders and potentiometers, position
  limiting and simple autonomous movement. Has a virtual takeInput() method
  implemented in subclasses */

#ifndef MOTOR_GROUP_INCLUDED
#define MOTOR_GROUP_INCLUDED

#include <API.h>

class PID;
class Timer;

class MotorGroup {
  public:
    void setPower(char power, bool overrideAbsolutes=false);
    /*Sets all motors in group to the specified power.
      If overrideAbsolutes is true, ignores absolute minimums and maximums */
    char getPower();  //returns the last set power of the motors in the group

    MotorGroup(unsigned char numMotors, unsigned char motors[]);
    MotorGroup(unsigned char numMotors, unsigned char motors[], unsigned char encPort1, unsigned char encPort2, double coeff=1);
    MotorGroup(unsigned char numMotors, unsigned char motors[], unsigned char potPort, bool potReversed=false);
    //sensors
    void addSensor(unsigned char encPort1, unsigned char encPort2, double coeff=1, bool setAsDefault=true); //associates a sensor with the group. If setAsDefault is true, potIsDefault is adjusted accordingly
    void addSensor(unsigned char potPort, bool reversed=false, bool setAsDefault=true);
    int encoderVal(bool rawValue=false);                                  //if encoder is attached, returns encoder value of associated encoder (multiplied by encCoeff unless rawValue is true), otherwise, it returns 0
    void resetEncoder();                                                  //resets associated encoder to 0
    int potVal();                                                         //same as encoderVal(), but returns 4095 - the value of the potentiometer if potReversed is true
    int getPosition();                                                    //returns either encoderVal() or potVal() depending on the value of potIsDefault
    //automovement
    void moveTowardPosition(int pos, char power=127);                                                                    //moves group toward specified position
    void goToPosition(int pos, bool runAsManeuver=false, char endPower=0, char maneuverPower=127, unsigned short timeout=10);  //goes to specified position
    void stopManeuver();
    void executeManeuver();                                                                                              //moves group toward target and updates maneuver progress
      //position targeting
    void posPIDinit(double kP, double kI, double kD, unsigned short minSampleTime=30, double integralMax=0, bool useTimeAdjustment=false);  //sets PID constants used for maintaining target position
    void setTargetPosition(int position); //sets target and activates position targeting
    void maintainTargetPos();							//moves toward or tries to maintain target position. posPIDinit() must have been called prior to this funciton
    bool errorLessThan(int margin);       //returns true if PID error < margin
    //accessors and mutators
      //sensors
    bool isPotReversed();       //returns false if no potentiometer is attached
    void setPotReversed(bool reversed);
    bool isPotDefault();
    void setDefaultSensor(bool potIsDefault);
    unsigned char getPotPort(); //returns port of associated potentiometer or 0 if no potentiometer is attached
    bool hasEncoder();
    bool hasPotentiometer();
      //automovement
    bool isManeuverExecuting();
    void activatePositionTargeting();
    void deactivatePositionTargeting();
      //position limits
    void setAbsMin(int minPos, char defPowerAtAbs=0, char maxPowerAtAbs=20);
    void setAbsMax(int maxPos, char defPowerAtAbs=0, char maxPowerAtAbs=20);
    void setAbsolutes(int minPos, int maxPos, char defPowerAtAbs=0, char maxPowerAtAbs=20); //set absMin and absMax simultaneously
    int getAbsMin();
    int getAbsMax();
    char getDefPowerAtAbs();
    void setDefPowerAtAbs(char power);
    char getMaxPowerAtAbs();
    void setMaxPowerAtAbs(char power);
  private:
    unsigned char numMotors;
    unsigned char* motors;      //array of motors in group
    //absolutes
    int absMin, absMax;         //the maximum and minimum potentiometer values for which the motor group will set motor powers above a certain threshold
    char maxPowerAtAbs;         //see below
    char defPowerAtAbs;         //the default absolute power assigned when motorGroup is set to a power greater than maxPowerAtAbs when potentiometer is outside of the range [absMin, absMax]
    bool hasAbsMax, hasAbsMin;  //whether absMin and absMax have been set
    //maneuvers - autonomous actions which can be run concurrently
    int maneuverTarget;             //encoder or potentiometer value maneuver tries to reach
    char maneuverPower, endPower;   //the motor powers during and after the maneuver
    unsigned short maneuverTimeout; //the amount of time (milliseconds) for which a position past the target position must be detected for maneuver to stop
    bool forward;                   //whether target is forward (in the positive motor power direction) of starting position
    bool maneuverExecuting;         //whether a maneuver is currently in progress
    Timer* maneuverTimer;           //tracks timeout state of maneuvers
		//position targeting
		PID* posPID;
    bool targetingActive;
    //sensors
    Encoder encoder;
    double encCoeff;
    unsigned char potPort;
    bool potReversed;   //whether potentiometer is reversed (affects potVal() output)
    bool potIsDefault;  //whether potentiometer (as opposed to encoder) is default sensor for position measurements
};


#endif
