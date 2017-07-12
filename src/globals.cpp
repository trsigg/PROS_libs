#include "config.h"

//drive
unsigned char leftMotors[] = ld_mps;
unsigned char rightMotors[] = rd_mps;
ParallelDrive drive(sizeof(leftMotors), sizeof(rightMotors), leftMotors, rightMotors);  //unsigned chars are 1 byte, so sizeof() == number of elements

//lift
unsigned char liftMotors[] = lift_mps;
ButtonGroup lift(liftGroup, sizeof(liftMotors), liftMotors);
