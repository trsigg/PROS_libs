/* General class for modeling a parallel drive system

  Motors modeled by two JoystickGroups. Supports taking user input, managing and
  accessing JoystickGroup sensors as well as a gyroscope, position tracking, and
  autonomous movement. */

#ifndef PARALLEL_DRIVE_INCLUDED
#define PARALLEL_DRIVE_INCLUDED

#include "coreIncludes.h" //also includes cmath
#include <vector>
#include <API.h>

class JoystickGroup;
class PID;
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
  unsigned short waitAtEnd, sampleTime, brakeDuration;
  double rampConst1, rampConst2, rampConst3;  //initialPower/0; maxPower/kP; finalPower/kD
} tDefs;

struct {
  correctionType defCorrectionType;
  bool rawValue;                             //whether to use encoder clicks (as opposed to inches)
  char brakePower;
  unsigned short waitAtEnd, sampleTime, brakeDuration, timeout;
  double rampConst1, rampConst2, rampConst3; //same as turn
  double kP_c, kI_c, kD_c;                   //correction PID constants
  double minDiffPerSample;                           //minimum speed (inches or clicks per second) which will not trigger a timeout
} dDefs;
//#endregion

class ParallelDrive {
  public:
    //#region main methods
    void takeInput();
    void setLeftPower(char power);
    void setRightPower(char power);
    void setDrivePower(char left, char right);
    //#endregion

    //#region constructors
    //remove? ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors);
    ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, Encoder leftEnc, Encoder rightEnc, double wheelDiameter, double gearRatio=1);
    ParallelDrive(std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff=1, double powMap=1, unsigned char maxAcc100ms=0, unsigned char deadband=10, unsigned char leftAxis=3, unsigned char rightAxis=2, unsigned char joystick=1); //configures tank input
    ParallelDrive(unsigned char movementAxis, unsigned char turningAxis, std::vector<unsigned char> leftMotors, std::vector<unsigned char> rightMotors, double coeff=1);  //configures arcade input
    //#endregion

    //#region input config
    void configureTankInput(double coeff=1, double powMap=1, unsigned char maxAcc100ms=0, unsigned char deadband=10, unsigned char leftAxis=3, unsigned char rightAxis=2, unsigned char joystick=1);
    void configureArcadeInput(unsigned char movementAxis=1, unsigned char turningAxis=2, double coeff=1);
    //#endregion
    //#region sensors
    void addSensor(Encoder encoder, encoderConfig side, double wheelDiameter=3.25, double gearRatio=1); //encCoeff calculated from diameter and gear ratio (from wheel to encoder)
    void addSensor(Gyro gyro, gyroCorrectionType correction=MEDIUM, bool setAbsAngle=true);
    double encoderVal(encoderConfig side=UNASSIGNED, bool rawValue=false, bool absolute=true);
    /* Returns the result of calling encoderVal() on motor group of specified
        side. When side is UNASSIGNED, encConfig is used to determine which
        encoders to use. AVERAGE returns the mean of the two sides' values, or
        that of their absolute values if absolute is true. */
    void resetEncoders();                       //When side is UNASSIGNED, encConfig is used to determine which encoder to reset
    double gyroVal(angleType format=DEGREES);
    void resetGyro();
    double absAngle(angleType format=DEGREES);  //gyroVal() + angleOffset
    //#endregion
    //#region position tracking
    void updatePosition();  //takes encoder (and possibly gyro) input and updates robot's current position
    double calculateWidth(unsigned short duration=10000, unsigned short sampleTime=200, char power=80, unsigned short reverseDelay=750);
    /* Causes robot to spin and uses gyro and encoder input to calculate width
        of its drive, which is returned and automatically set. Power is the
        motor power used in turning, and reverse delay is the amount of time for
        which samples are not taken as drive changes spinning direction. */
    //#endregion
    //#region automovement
    void turn(double angle, bool runAsManeuver=false, double in1=tDefs.rampConst1, double in2=tDefs.rampConst2, double in3=tDefs.rampConst3, angleType format=tDefs.defAngleType, unsigned short waitAtEnd=tDefs.waitAtEnd, unsigned short sampleTime=tDefs.sampleTime, char brakePower=tDefs.brakePower, unsigned short brakeDuration=tDefs.brakeDuration, bool useGyro=tDefs.useGyro);
    void drive(double dist, bool runAsManeuver=false, double in1=dDefs.rampConst1, double in2=dDefs.rampConst2, double in3=dDefs.rampConst3, unsigned short waitAtEnd=dDefs.waitAtEnd, double kP=dDefs.kP_c, double kI=dDefs.kI_c, double kD=dDefs.kD_c, correctionType correction=dDefs.defCorrectionType, bool rawValue=dDefs.rawValue, double minSpeed=dDefs.minDiffPerSample, unsigned short timeout=dDefs.timeout, char brakePower=dDefs.brakePower, unsigned short brakeDuration=dDefs.brakeDuration, unsigned short sampleTime=dDefs.sampleTime);
    /* For quad-ramped movement, in1=initial power, in2=maxPower, and
        in3=finalPower. For PD-ramped movement, in1=0, in2=kP, and in3=kD. */
    void executeManeuver();                             //executes turn and drive maneuvers
    double maneuverProgress(angleType format=DEGREES);  //returns absolute value odistance traveled or angle turned while maneuver is in progress
    bool maneuverExecuting();
    //#endregion
    //#region accessors and mutators
      //#subregion sensors
    void setEncoderConfig(encoderConfig config);
    void setAbsAngle(double angle=0, angleType format=DEGREES); //sets angleOffset so that current absAngle is equal to specified angle
    Gyro* getGyroPtr();
    bool hasGyro();
      //#endsubregion
      //#subregion position tracking
    void setWidth(double inches);
    void setRobotPosition(double x, double y, double theta, angleType format=DEGREES, bool updateAngleOffset=true); //sets angleOffset so that current absAngle is equal to theta if setAbsAngle if true
    double x(); double y(); double theta(angleType format=DEGREES);
      //#endsubregion
      //#subregion autonomous
    void setCorrectionType(correctionType type);
      //#endsubregion
    //#endregion
  private:
    JoystickGroup* leftDrive;
    JoystickGroup* rightDrive;
    //#region arcade
    bool arcadeInput;
    double coeff;
    unsigned char moveAxis, turnAxis, joystick;
    //#endregion
    //#region sensors
    void updateEncConfig(); //automatically updates encConfig when a new encoder is attached
    encoderConfig encConfig;
    Gyro* gyro;
    int angleOffset;  //amount added to gyro values to obtain absolute angle (degrees)
    //#endregion
    //#region position tracking
    double xPos, yPos, orientation; //orientation is in radians
    double width;                   //width of drive in inches (wheel well to wheel well)
    Timer* positionTimer;
    unsigned short minSampleTime; //minimum time between updates of robot's position
    gyroCorrectionType gyroCorrection;
    //#endregion
    //#region automovement
    void initializeDefaults();  //initializes default automovement values (called by constructors)
    double target;  //angle or distance
    Ramper* ramp;    //controls motor power ramping during maneuver
    unsigned short finalDelay, sampleTime, brakeDelay;
    char brakePower;
      //#subregion turning
    bool isTurning;
    angleType format;
    bool usingGyro;
      //#endsubregion
      //#subregion driving
    bool isDriving;
    bool rawValue;
    double minDiffPerSample;
    unsigned short timeout;
    correctionType correction;
    PID* correctionPID;
    double leftDist, rightDist, totalDist;
    Timer* sampleTimer;
    Timer* timeoutTracker;
      //#endsubregion
    //#endregion
};

#endif
