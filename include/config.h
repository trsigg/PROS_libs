/* Global configuration variables */

#ifndef CONFIG_INCLUDED
#define CONFIG_INCLUDED

#define TEST_BOT

//#region defaults
#define NUM_JOYSTICKS 1
#define DEBUG_LEVEL 0 //0 is not debugging, higher levels yield increasingly detailed output (currently non-functional)
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
  //flapper
  #define FLAPPER_MPS  { 10 }
  //#endsubregion
  //#subregion sensors
  //flapper
  #define FLAPPER_POT  1
  //drive
  #define HYRO        5
  #define RIGHT_ENC   4, 3, false
  #define LEFT_ENC    2, 1, false
  //#endsubregion
  //#subregion buttons
  //flapper
  #define FLAPPER_GROUP 6
  //#endsubregion
  //#subregion constants
  #define WHEEL_DIAMETER 4.0
  //#endsubregion
  //#region global externs
  extern ParallelDrive drive;
  extern ButtonGroup flapper;
  //#endregion
#endif
//#endregion

#endif
