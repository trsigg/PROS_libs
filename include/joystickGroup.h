/* Extension of MotorGroup with pre-coded joystick control */

#ifndef JOYSTICK_GROUP_INCLUDED
#define JOYSTICK_GROUP_INCLUDED

#include "motorGroup.h"

class JoystickGroup : public MotorGroup {
  public:
    //Reads input from controller and sets group power accordingly. Returns power of group.
    char takeInput();
    //sets up joystick
    void configureInput(unsigned char axis, double coeff=1, double powMap=1, unsigned char maxAcc100ms=0, unsigned char deadband=10, unsigned char joystick=1);

    JoystickGroup(unsigned char numMotors, unsigned char motors[]);
    JoystickGroup(unsigned char numMotors, unsigned char axis, unsigned char motors[], double coeff=1, double powMap=1, unsigned char maxAcc100ms=60, unsigned char deadband=10, unsigned char joystick=1);
    JoystickGroup(unsigned char numMotors, unsigned char motors[], unsigned char encPort1, unsigned char encPort2, double coeff=1);
    JoystickGroup(unsigned char numMotors, unsigned char motors[], unsigned char potPort, bool potReversed=false);

    //accessors and mutators
      //state
    bool isActive();
    void activate();  //does not check if input is correctly configured, so use with caution
    void deactivate();
      //ramping
    bool isRamped();
    void startRamping();  //does not change msPerPowerChange if it was previously specified
    void startRamping(unsigned char maxAcc100ms);
    void stopRamping();
  private:
    unsigned char axis, joystick;
    unsigned char deadband; //magnitude of the range of motor values around 0 for which motors are not engaged
    double powMap;          //degree of polynomial to which inputs are mapped (1 for linear)
    double coeff;           //factor by which motor powers are multiplied (after all other calculations)
    bool active;            //whether group is accepting input
    //ramping
    bool ramping;
    unsigned char msPerPowerChange;
    unsigned long lastUpdated;
};

#endif
