#include "main.h"

using namespace pros;

#ifndef GLOBALS
#define GLOBALS

/*
    only put declarations in this file
*/

extern pros::Controller master;
extern pros::Controller partner;
extern pros::Motor BackR, MiddleR, FrontR, BackL, MiddleL, FrontL;
extern pros::Motor Arm1;
extern pros::ADIDigitalOut pistonF, pistonB1, pistonB2;
extern pros::Motor Elev;
extern pros::Gps gps1;
extern pros::Gps gps2;
extern pros::Imu Inertial;

extern pros::Motor Left, Right;

extern int autonomousPreSet;

#endif