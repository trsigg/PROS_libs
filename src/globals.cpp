#include "config.h"

//drive
static unsigned char leftMotorArray[] = ld_mps;
static unsigned char rightMotorArray[] = rd_mps;
static std::vector<unsigned char> leftMotors(leftMotorArray, leftMotorArray + sizeof(leftMotorArray) / sizeof(unsigned char));
static std::vector<unsigned char> rightMotors(rightMotorArray, rightMotorArray + sizeof(rightMotorArray) / sizeof(unsigned char));
ParallelDrive drive(leftMotors, rightMotors);

//lift
static unsigned char liftMotorArray[] = lift_mps;
static std::vector<unsigned char> liftMotors(liftMotorArray, liftMotorArray + sizeof(liftMotorArray) / sizeof(unsigned char));
ButtonGroup lift(liftGroup, liftMotors, -10);

//claw
static unsigned char clawLarray[] = clawR_mps;
static unsigned char clawRarray[] = clawL_mps;
static std::vector<unsigned char> clawLmotors(clawLarray, clawLarray + sizeof(clawLarray) / sizeof(unsigned char));
static std::vector<unsigned char> clawRmotors(clawRarray, clawRarray + sizeof(clawRarray) / sizeof(unsigned char));
MotorGroup clawL(clawLmotors);
MotorGroup clawR(clawRmotors);
