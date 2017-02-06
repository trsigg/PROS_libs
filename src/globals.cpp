#include "config.h"
#include <algorithm>

//drive
unsigned char leftMotors[] = ld_mps;
unsigned char rightMotors[] = rd_mps;
ParallelDrive drive(2 * std::min(sizeof(leftMotors), sizeof(rightMotors))/sizeof(unsigned char), leftMotors, rightMotors);  //min ensures no overflow

//lift
unsigned char liftMotors[] = lift_mps;
ButtonGroup lift(sizeof(liftMotors)/sizeof(unsigned char), liftGroup, liftMotors, -10);

//claw
unsigned char clawLmotors[] = clawR_mps;
unsigned char clawRmotors[] = clawL_mps;
MotorGroup clawL(sizeof(clawLmotors)/sizeof(unsigned char), clawLmotors);
MotorGroup clawR(sizeof(clawRmotors)/sizeof(unsigned char), clawRmotors);
