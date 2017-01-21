/* Extension of MotorGroup with pre-coded button control */

#ifndef BUTTON_GROUP_INCLUDED
#define BUTTON_GROUP_INCLUDED

#include "motorGroup.h"

class ButtonGroup : public MotorGroup {
  public:
    //Reads input from joystick and sets group power accordingly. Returns power of group.
    int takeInput();
    //input configuration
    void configureInput(unsigned char buttonGroup, char stillSpeed=0, unsigned char buttonConfig=0, char power=127, char downPower=0, unsigned char joystick=1);  //standard button input
    /*buttonConfig determines which button combination from the specified group is used.
      Its possible values are shown in the table at the bottom of this file.

      If downPower is zero, it will be set to -upPower. Even when specified, it
      will always be given a sign opposite to that of upPower. */
    void configureMultiGroupInput(unsigned char upGroup, unsigned char upButton, unsigned char downGroup, unsigned char downButton, char stillSpeed=0, char power=127, char downPower=0, unsigned char upJoystick1=1, unsigned char downJoystick=1); //input system with buttons from multiple groups

    ButtonGroup(std::vector<unsigned char> motors);
    ButtonGroup(std::vector<unsigned char> motors, unsigned char buttonGroup, char stillSpeed=0, unsigned char buttonConfig=0, char power=127, char downPower=0, unsigned char joystick=1);
    ButtonGroup(std::vector<unsigned char> motors, Encoder* encoder);
    ButtonGroup(std::vector<unsigned char> motors, unsigned char potPort, bool potReversed=false);

    //accessors and mutators
      //state
    bool isActive();
    void activate();  //Does not check if input is correctly configured, so use with caution
    void deactivate();
      //motor powers
    char getStillSpeed();
    char getUpPower();
    char getDownPower();
    void setStillSpeed(char power);
    void setMovementPower(char power, char downPower=0);  //obeys same behavior as constructor
  private:
    //input buttons
    unsigned char upJoystick, upGroup, upButton;
    unsigned char downJoystick, downGroup, downButton;
    //motor powers when up button, down button, and no buttons are pressed
    char upPower, downPower, stillSpeed;
    //whether group is accepting input
    bool active;
};

#endif

/*******BUTTON CONFIGURATIONS*******
| Number | Up button | Down button |
|--------|-----------|-------------|
|    0   |     U     |     D       |
|    1   |     L     |     R       |
|    2   |     U     |     L       |
|    3   |     U     |     R       |
|    4   |     L     |     D       |
|    5   |     R     |     D       |
|________|___________|_____________|
*/
