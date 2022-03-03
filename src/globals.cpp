#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor BackR (1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MiddleR (2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor FrontR (3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BackL (4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MiddleL (6, pros ::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor FrontL (5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Arm1 (7, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Elev (9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Gps gps1(19, -0.0508, 0.17272);
// 2 X position from center
// 6.8 Y position from center 
pros::Gps gps2(20, 0.1905, -0.1397);
//9.5 inches is hypotenuse of the triangle
//-7.75 X position from center
//5.5 Y position from center
pros::Imu Inertial(15);
pros::ADIDigitalOut pistonF('A');
pros::ADIDigitalOut pistonB1('B');
pros::ADIDigitalOut pistonB2('C');