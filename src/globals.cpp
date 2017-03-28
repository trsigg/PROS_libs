#include "config.h"
#include <algorithm>

//drive
unsigned char leftMotors[] = ld_mps;
unsigned char rightMotors[] = rd_mps;
ParallelDrive drive(2 * std::min(sizeof(leftMotors), sizeof(rightMotors))/sizeof(unsigned char), leftMotors, rightMotors);  //min ensures no overflow

//lift
unsigned char liftMotors[] = lift_mps;
ButtonGroup lift(liftGroup, sizeof(liftMotors)/sizeof(unsigned char), liftMotors, -10);

//claw
unsigned char clawLmotors[] = clawL_mps;
unsigned char clawRmotors[] = clawR_mps;
MotorGroup clawL(sizeof(clawLmotors)/sizeof(unsigned char), clawLmotors);
MotorGroup clawR(sizeof(clawRmotors)/sizeof(unsigned char), clawRmotors);
