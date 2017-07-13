#include "config.h"

//drive
unsigned char leftMotors[] = LD_MPS;
unsigned char rightMotors[] = RD_MPS;
ParallelDrive drive(sizeof(leftMotors), sizeof(rightMotors), leftMotors, rightMotors);  //unsigned chars are 1 byte, so sizeof() == number of elements

//flapper
unsigned char flapperMotors[] = FLAPPER_MPS;
ButtonGroup flapper(FLAPPER_GROUP, sizeof(flapperMotors), flapperMotors);
