/* General class for modeling a parallel drive system

  Motors modeled by two JoystickGroups. Supports taking user input, managing and
  accessing JoystickGroup sensors as well as a gyroscope, position tracking, and
  autonomous movement. */

#ifndef PARALLEL_DRIVE_INCLUDED
#define PARALLEL_DRIVE_INCLUDED

#include "coreIncludes.h" //also includes <cmath>
#include <vector>

class JoystickGroup;
class Position;
class Timer;
class Encoder;
class Gyro;

enum encoderConfig { UNASSIGNED, LEFT, RIGHT, AVERAGE };
/* How encoderVal() (which is used in many internal functions in addition to
    being accessible to user) is calculated by default.

    LEFT and RIGHT only use values from one side of the drive, AVERAGE uses
    their mean. */
enum gyroCorrectionType { NONE, MEDIUM, FULL };
/* How gyro is used during position tracking.

    NONE does not use gyro. MEDIUM (currently most reliable) uses gyro to track
    orientation only. FULL attempts to correct encoder values based on gyro input. */

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

    //defaults
    struct turnDefaults;
    struct driveDefaults;
    //sensors
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
    //position tracking
    void updatePosition();  //takes encoder (and possibly gyro) input and updates robot's current position
    //misc
    double calculateWidth(int duration=10000, int sampleTime=200, int power=80, int reverseDelay=750);
    /* Causes robot to spin and uses gyro and encoder input to calculate width
        of its drive, which is returned but NOT automatically set. Power is the
        motor power used in turning, and reverse delay is the amount of time for
        which samples are not taken as drive changes spinning direction. */
    //accessors and mutators
      //sensors
    void setEncoderConfig(encoderConfig config);
    void setAbsAngle(double angle=0, angleType format=DEGREES); //sets angleOffset so that current absAngle is equal to specified angle
      //position tracking
    void setRobotPosition(double x, double y, double theta, bool setAbsAngle=true); //sets angleOffset so that current absAngle is equal to theta if setAbsAngle if true
  private:
    void updateEncoderConfig(); //automatically updates encConfig when a new encoder is attached
    JoystickGroup* leftMotors;
    JoystickGroup* rightMotors;
    Position* robotPosition;
    double width; //width of drive in inches (wheel well to wheel well). Used to track position.
    //position tracking
    Timer* positionTimer;
    unsigned short minSampleTime; //minimum time between updates of robot's position
    gyroCorrectionType gyroCorrection;
    //sensors
    encoderConfig encConfig;
    Gyro* gyro;
    int angleOffset;  //amount added to gyro values to obtain absolute angle
};

#endif
