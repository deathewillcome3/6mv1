#include "main.h"
#include "algos.h"
#include "daniel_auton.h"
#include "pranav_soph_auton.h"
#include "drive.h"
#include <math.h>
#include <vector>


// class pi_c{
//   public:
//     double p = 0.5;
//     double i = 5;s
//     double integral = 0;
//     double previous_time = vex::timer::system();
//     double update(double delta){
//       double dt = vex::timer::system() - previous_time;
//       previous_time = vex::timer::system();
//       integral = integral * 0.9999 + delta * (dt / 1000);
//       return p * delta + i * integral;
//     };
// };



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::Motor FrontL (8, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor FrontR (6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor BackL (4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor BackR (13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor BackR (1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    // pros::Motor MiddleR (2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    // pros::Motor FrontR (3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    // pros::Motor BackL (4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    // pros::Motor MiddleL (6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    // pros::Motor FrontL (5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor Arm1 (7, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor Elev (8, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	//This is a test to see if commits work
	pros::ADIDigitalOut piston1('A');
	pros::ADIDigitalOut piston2('B');
	pros::IMU Inertial(5);
	pros::ADIDigitalOut piston3('C');
// inertial Inertial= inertial(12);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor BackL(4);
	pros::Motor MiddleL(6);
	pros::Motor FrontL(5);
	pros::Motor BackR(1);
	pros::Motor MiddleR(2);
	pros::Motor FrontR(3);
	pros::Motor Arm1(7);
	pros::Motor Arm2(5);
	// pros::Motor Elev(8);
	pros::Motor Elev(9);
	pros::ADIDigitalOut piston1('A');
	pros::ADIDigitalOut piston2('B');
	pros::ADIDigitalOut piston3('C');
	pros::Imu Inertial(15);
	pros::Gps gps1(15);
	pros::Gps gps2(18);
	pros::c::gps_status_s_t status;
	pros::c::gps_status_s_t status1;
	float arm_deg = 0;
	Inertial.reset();
	// pi_c flp;
	// pi_c frp;
	// pi_c blp;
	// pi_c brp;
	while (true) {
		// Arm1.move_velocity(18);
		double a4 ;
		double a3 ;
		if (a4 < 63.5 && a4 > -63.5){
			a4 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*2;
		}
		else {
			a4 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) ;
		}
		if (a3 < 63.5 && a3 > -63.5){
			a3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 2;
		}
		else{
			a3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) ;
		}
		//a4 is the x coordinate
		//a3 is the y coordinate
	
		double leftMovement = a4+a3;
		//right movement: -a4+a3
		//except they turn opposite so multiply by -1
		double rightMovement = a4-a3;

		FrontL.move(-1*leftMovement);
		FrontR.move(-1*rightMovement);
		//Back moters r literally just the negative of the front motors 
		MiddleL.move(-1*leftMovement);
		MiddleR.move(-1*rightMovement);
		BackL.move(-1*leftMovement);
		BackR.move(-1*rightMovement);

		status = gps1.get_status();

		pros::screen::print(TEXT_MEDIUM, 1, "X Position: %3f", status.x);
		pros::screen::print(TEXT_MEDIUM, 2, "Y Position: %3f", status.y);
		pros::screen::print(TEXT_MEDIUM, 3, "Pitch: %3f", status.pitch);
		pros::screen::print(TEXT_MEDIUM, 4, "Roll: %3f", status.roll);
		pros::screen::print(TEXT_MEDIUM, 5, "Yaw: %3f", status.yaw);

		
		status1 = gps2.get_status();

		pros::screen::print(TEXT_MEDIUM, 6, "X Position (2): %3f", status1.x);
		pros::screen::print(TEXT_MEDIUM, 7, "Y Position (2): %3f", status1.y);
		pros::screen::print(TEXT_MEDIUM, 8, "Pitch (2): %3f", status1.pitch);
		pros::screen::print(TEXT_MEDIUM, 9, "Roll (2): %3f", status1.roll);
		pros::screen::print(TEXT_MEDIUM, 10, "Yaw (2): %3f", status1.yaw);
	
		arm_deg = arm_deg + Arm1.get_actual_velocity()/600;
		if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 6){
			
			if (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) > 0 && arm_deg > 60){
				Arm1.move(0);
			}
			Arm1.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		}
		else {
			Arm1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			Arm1.move_velocity(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			piston2.set_value(false);
			piston3.set_value(false);

		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			piston2.set_value(true);
			piston3.set_value(true);
		}
		else{
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			piston1.set_value(false);

		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			piston1.set_value(true);

		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			Elev.move(0);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			daniel_auton();
		} 
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			piston1.set_value(false);
		} 
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			Elev.move(127);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			Elev.move(-127);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			pranav_soph_auton();
		}
		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
		// 	// autonomous5();
		// }
		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
		// 	// autonomous6();
		// }
		pros::delay(20);
	}
}

