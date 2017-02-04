/* General class for modeling a parallel drive system

  Motors modeled by two JoystickGroups. Supports taking user input, managing and
  accessing JoystickGroup sensors as well as a gyroscope, position tracking, and
  autonomous movement. */

#ifndef PARALLEL_DRIVE_INCLUDED
#define PARALLEL_DRIVE_INCLUDED

#include "coreIncludes.h" //also includes <cmath>
#include <vector>

class Encoder;
class Gyro;
class JoystickGroup;
class PID;
class Position;
class Ramper;
class Timer;

//#region enums
enum encoderConfig { UNASSIGNED, LEFT, RIGHT, AVERAGE };
/* How encoderVal() (which is used in many internal functions in addition to
    being accessible to user) is calculated by default.

    LEFT and RIGHT only use values from one side of the drive, AVERAGE uses
    their mean. */
enum gyroCorrectionType { NO, MEDIUM, FULL };
/* How gyro is used during position tracking.

    NONE does not use gyro. MEDIUM (currently most reliable) uses gyro to track
    orientation only. FULL attempts to correct encoder values based on gyro input. */
enum correctionType { NONE, GYRO, ENCODER, AUTO };
/* How correction is performed during automovement. GYRO tries to maintain a
    gyro value of 0, ENCODER tries to maintain an difference in the drive side
    encoder counts of zero, and AUTO will cause the program to choose
    automatically based on the available sensors (encoders given preference) */
//#endregion

//#region defaults
struct {
  angleType defAngleType;
  bool useGyro;
  char brakePower;
  unsigned short waitAtEnd, brakeDuration;
  double rampConst1, rampConst2, rampConst3;  //initialPower/0; maxPower/kP; finalPower/kD
} tDefs;

struct {
  correctionType defCorrectionType;
  bool rawValue;                             //whether to use encoder clicks (as opposed to inches)
  char brakePower;
  unsigned short waitAtEnd, sampleTime, brakeDuration, timeout;
  double rampConst1, rampConst2, rampConst3; //same as turn
  double kP_c, kI_c, kD_c;                   //correction PID constants
  double minSpeed;                           //minimum speed (inches or clicks per second) which will not trigger a timeout
} dDefs;
//#endregion

class ParallelDrive {
  public:
    void takeInput();
    void setLeftPower(char power);
    void setRightPower(char power);
    void setDrivePower(char left, char right);
    void configureTankInput(double coeff=1, double powMap=1, unsigned char maxAcc100ms=0, unsigned char deadband=10, unsigned char joystick=1);
    void configureArcadeInput(unsigned char movementAxis, unsigned char turningAxis, double coeff=1);

    ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, Encoder* leftEnc, Encoder* rightEnc=nullptr);
    ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff=1, double powMap=1, unsigned char maxAcc100ms=0, unsigned char deadband=10, unsigned char leftAxis=3, unsigned char leftJoystick=1, unsigned char rightAxis=2); //configures tank input
    ParallelDrive(unsigned char movementAxis, unsigned char turningAxis, std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff=1);  //configures arcade input

    //#region sensors
    void addSensor(Encoder encoder, encoderConfig side, double wheelDiameter=3.25, double gearRatio=1); //encCoeff calculated from diameter and gear ratio (from wheel to encoder)
    void addSensor(Gyro gyro, gyroCorrectionType correction=MEDIUM, bool setAbsAngle=true);
    double encoderVal(encoderConfig side=UNASSIGNED, bool rawValue=false, bool absolute=true);
    /* Returns the result of calling encoderVal() on motor group of specified
        side. When side is UNASSIGNED, encConfig is used to determine which
        encoders to use. AVERAGE returns the mean of the two sides' values, or
        that of their absolute values if absolute is true. */
    void resetEncoder(encoderConfig side=UNASSIGNED); //When side is UNASSIGNED, encConfig is used to determine which encoder to reset
    double gyroVal(angleType format=DEGREES);
    void resetGyro();
    double absAngle(angleType format=DEGREES);        //gyroVal() + angleOffset
    //#endregion
    //#region position tracking
    void updatePosition();  //takes encoder (and possibly gyro) input and updates robot's current position
    //#endregion
    //#region misc
    double calculateWidth(int duration=10000, int sampleTime=200, int power=80, int reverseDelay=750);
    /* Causes robot to spin and uses gyro and encoder input to calculate width
        of its drive, which is returned but NOT automatically set. Power is the
        motor power used in turning, and reverse delay is the amount of time for
        which samples are not taken as drive changes spinning direction. */
    //#endregion
    //#region automovement
    void turn(double angle, bool runAsManeuver=false, double in1=tDefs.rampConst1, double in2=tDefs.rampConst2, double in3=tDefs.rampConst3, angleType format=tDefs.defAngleType, unsigned short waitAtEnd=tDefs.waitAtEnd, char brakePower=tDefs.brakePower, unsigned short brakeDuration=tDefs.brakeDuration, bool useGyro=tDefs.useGyro); //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
    void drive(double dist, bool runAsManeuver=false, double in1=dDefs.rampConst1, double in2=dDefs.rampConst2, double in3=dDefs.rampConst3, double kP=dDefs.kP_c, double kI=dDefs.kI_c, double kD=dDefs.kD_c, correctionType correction=dDefs.defCorrectionType, bool rawValue=dDefs.rawValue, double minSpeed=dDefs.minSpeed, unsigned short timeout=dDefs.timeout, char brakePower=dDefs.brakePower, unsigned short waitAtEnd=dDefs.waitAtEnd, unsigned short sampleTime=dDefs.sampleTime); //for PD, in1=0, in2=kP, in3=kD; for quad ramping, in1=initial, in2=maximum, and in3=final
    void executeDriveManeuver();  //executes turn and drive maneuvers
    void driveManeuverProgress(); //returns distance traveled or angle turned while maneuver is in progress
    //#endregion
    //#region accessors and mutators
      //#subregion sensors
    void setEncoderConfig(encoderConfig config);
    void setAbsAngle(double angle=0, angleType format=DEGREES); //sets angleOffset so that current absAngle is equal to specified angle
      //#endsubregion
      //#subregion position tracking
    void setRobotPosition(double x, double y, double theta, bool setAbsAngle=true); //sets angleOffset so that current absAngle is equal to theta if setAbsAngle if true
      //#endsubregion
    //#endregion
  private:
    void updateEncoderConfig(); //automatically updates encConfig when a new encoder is attached
    JoystickGroup* leftMotors;
    JoystickGroup* rightMotors;
    Position* robotPosition;
    double width; //width of drive in inches (wheel well to wheel well). Used to track position.
    //#region sensors
    encoderConfig encConfig;
    Gyro* gyro;
    int angleOffset;  //amount added to gyro values to obtain absolute angle
    //#endregion
    //#region position tracking
    Timer* positionTimer;
    unsigned short minSampleTime; //minimum time between updates of robot's position
    gyroCorrectionType gyroCorrection;
    //#endregion
    //#region automovement
    double target;  //angle or distance
    Ramper* ramp;    //controls motor power ramping during maneuver
    unsigned short waitAtEnd, brakeDuration;
    char brakePower;
    bool driveManeuverExecuting;
    bool forward;   //sign of target
      //#subregion turning
    angleType format;
    bool usingGyro;
      //#endsubregion
      //#subregion driving
    bool rawValue;
    double minSpeed;
    unsigned int sampleTime, timeout;
    correctionType correction;
    PID* correctionPID;
    double leftDist, rightDist, totalDist;
    Timer* driveTimer;
      //#endsubregion
    void setCorrectionType();
    //#endregion
};

#endif
