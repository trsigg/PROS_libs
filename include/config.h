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
  #define ld_mps  { 1 }   //left
  #define rd_mps  { 9 }  //right
  //lift
  #define lift_mps  { 10 }
  //#endsubregion
  //#subregion sensors
  //lift
  #define liftPot   1
  //drive
  #define hyro      5
  #define rightEnc  3, 4, false
  #define leftEnc   1, 2, false
  //#endsubregion
  //#subregion buttons
  //lift
  #define liftGroup 5
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
