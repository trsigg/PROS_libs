/* Global configuration variables */

#ifndef CONFIG_INCLUDED
#define CONFIG_INCLUDED

#define TEST_BOT

//#region defaults
#define NUM_JOYSTICKS 1
#define DEBUG_LEVEL 0 //0 is not debugging, higher levels yield increasingly detailed output (currently non-functional)
#define DEF_GYRO_MULTIPLIER 196
//#endregion


//#region Test bot
#ifdef TEST_BOT
  //#subregion includes
  #include "parallelDrive.h"
  #include "buttonGroup.h"
  //#endsubregion
  //#subregion motors
  //initializer lists of motor ports (mps = motor ports)
  //drive
  #define LD_MPS  { 1 }   //left
  #define RD_MPS  { 9 }  //right
  //lift
  #define LIFT_MPS  { 10 }
  //#endsubregion
  //#subregion sensors
  //lift
  #define LIFT_POT  1
  //drive
  #define HYRO      5
  #define RIGHT_ENC 3, 4, false
  #define LEFT_ENC  1, 2, false
  //#endsubregion
  //#subregion buttons
  //lift
  #define LIFT_GROUP 6
  //#endsubregion
  //#subregion constants
  #define WHEEL_DIAMETER 4.0
  //#endsubregion
  //#region global externs
  extern ParallelDrive drive;
  extern ButtonGroup lift;
  //#endregion
#endif
//#endregion

#endif
