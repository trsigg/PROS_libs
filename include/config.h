/* Global configuration variables */

#ifndef CONFIG_INCLUDED
#define CONFIG_INCLUDED

#define E_TEAM

//#region defaults
#define NUM_JOYSTICKS 1
#define DEBUG_LEVEL 0 //0 is not debugging, higher levels yield increasingly detailed output (currently non-functional)
#define DEF_GYRO_MULTIPLIER 196
//#endregion


//#region E-team
#ifdef E_TEAM
  //#subregion includes
  #include "parallelDrive.h"
  #include "buttonGroup.h"
  //#endsubregion
  //#subregion motors
  //initializer lists of motor ports (mps = motor ports)
  //drive
  #define ld_mps  { 1, 5 }      //left
  #define rd_mps  { 3, 8, 10 }  //right
  //lift
  #define lift_mps  { 2, 6, 9 }
  //claw
  #define clawR_mps { 4 }
  #define clawL_mps { 7 }
  //#endsubregion
  //#subregion sensors
  //lift
  #define liftPot   2
  //claw
  #define clawPotR  3
  #define clawPotL  4
  //drive
  #define hyro      1
  #define rightEnc  1, 2, false
  #define leftEnc   3, 4, false
  //autonomous
  #define sidePot   5
  #define modePot   6
  //#endsubregion
  //#subregion buttons
  //lift
  #define liftGroup 5
  //claw
  #define openClawBtn     1, 6, JOY_UP
  #define closeClawBtn    1, 6, JOY_DOWN
  #define hyperextendBtn  1, 7, JOY_RIGHT
  //toggle
  #define toggleAutoDumpGroup 8
  #define toggleClawModeGroup 7
  //#endsubregion
  //#subregion constants
  #define WHEEL_DIAMETER 2.75
  //#endsubregion
#endif
//#endregion

//#region general
  //#subregion globals
  std::vector<unsigned char> leftMotors = ld_mps;
  std::vector<unsigned char> rightMotors = rd_mps;
  std::vector<unsigned char> liftMotors = lift_mps;
  std::vector<unsigned char> clawLmotors = clawL_mps;
  std::vector<unsigned char> clawRmotors = clawR_mps;

  ParallelDrive drive(leftMotors, rightMotors);

  ButtonGroup lift(liftGroup, liftMotors, -10);

  MotorGroup clawL(clawLmotors);
  MotorGroup clawR(clawRmotors);
  //#endsubregion
//#endregion

#endif
