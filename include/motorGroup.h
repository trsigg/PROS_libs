/* General class for modeling a single block of motors as a group

  Supports setting motors, associating encoders and potentiometers, position
  limiting and simple autonomous movement. Has a virtual takeInput() method
  implemented in subclasses */

#ifndef MOTOR_GROUP_INCLUDED
#define MOTOR_GROUP_INCLUDED

#include <vector>
#include "API.h"

class PID;
class Timer;

class MotorGroup {
  public:
    void setPower(char power, bool overrideAbsolutes=false);
    /*Sets all motors in group to the specified power.
      If overrideAbsolutes is true, ignores absolute minimums and maximums */
    char getPower();  //returns the last set power of the motors in the group

    MotorGroup(std::vector<unsigned char> motors);  //TODO: should these vectors be passed as pointers?
    MotorGroup(std::vector<unsigned char> motors, Encoder* encoder, double coeff=1);
    MotorGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed=false);
    //sensors
    void addSensor(Encoder* enc, double coeff=1, bool setAsDefault=true); //associates a sensor with the group. If setAsDefault is true, potIsDefault is adjusted accordingly
    void addSensor(unsigned char port, bool reversed=false, bool setAsDefault=true);
    int encoderVal(bool rawValue=false);                                  //if encoder is attached, returns encoder value of associated encoder (multiplied by encCoeff unless rawValue is true), otherwise, it returns 0
    void resetEncoder();                                                  //resets associated encoder to 0
    int potVal();                                                         //same as encoderVal(), but returns 4095 - the value of the potentiometer if potReversed is true
    int getPosition();                                                    //returns either encoderVal() or potVal() depending on the value of potIsDefault
    //automovement
    void moveTowardPosition(int pos, char power=127);                                                       //moves group toward specified position
    void createManeuver(int position, char endPower=0, char maneuverPower=127, unsigned short timeout=10);  //sets a target position for group to attain
    void stopManeuver();
    void executeManeuver();                                                                                 //moves group toward target and updates maneuver progress
    void goToPosition(int pos, char endPower=0, char maneuverPower=127, unsigned short timeout=100);         //moves group to specified position
      //position targeting
    void setPosPIDconsts(double kP, double kI, double kD);  //sets PID constants used for maintaining target position
    void setTargetPosition(int position);                   //sets target and activates position targeting
    void maintainTargetPos();																//moves toward or tries to maintain target position. setPosPIDconsts() must have been called prior to this funciton
    //accessors and mutators
      //sensors
    bool isPotReversed();       //returns false if no potentiometer is attached
    void setPotReversed(bool reversed);
    bool isPotDefault();
    void setDefaultSensor(bool potIsDefault);
    Encoder* getEncoderPtr();   //returns pointer to associated encoder or nullptr if no encoder is attached
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
    std::vector<unsigned char> motors;  //vector (variable-length array) of motors in group
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
		PID* targetPosPID;
    bool targetingActive;
    //sensors
    Encoder* encoder;
    double encCoeff;
    unsigned char potPort;
    bool potReversed;   //whether potentiometer is reversed (affects potVal() output)
    bool potIsDefault;  //whether potentiometer (as opposed to encoder) is default sensor for position measurements
};


#endif
