#include "main.h"
#include "drive.h"
#include "algos.h"
#include "globals.h"


void move_base(double left, double right){
    // BackL = -left;
    // MiddleL = -left;
    // FrontL = -left;
    // BackR = -right;
    // MiddleR = -right;
    // FrontR = -right;

	BackL = -left;
    MiddleL = -left;
    FrontL = -left;
    BackR = right;
    MiddleR = right;
    FrontR = right;
}

void move_good(double revolutions){
    // BackL = -left;
    // MiddleL = -left;
    // FrontL = -left;
    // BackR = -right;
    // MiddleR = -right;
    // FrontR = -right;

	// BackL.move_relative(-300*revolutions,600);
	// MiddleL.move_relative(-300*revolutions,600);
	// FrontL.move_relative(-300*revolutions,600);
	// BackR.move_relative(300*revolutions,600);
	// MiddleR.move_relative(300*revolutions,600);
	// FrontR.move_relative(300*revolutions,600);


	BackL.move_relative(100,100);
	while (!((BackL.get_position() < -300*revolutions+5) && (BackL.get_position() > -300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}
	MiddleL.move_relative(100,100);
	FrontL.move_relative(100,100);
	BackR.move_relative(-100, 100);
	MiddleR.move_relative(-100, 100);
	FrontR.move_relative(-100, 100);

	

	/*
	while (!((MiddleL.get_position() < -300*revolutions+5) && (MiddleL.get_position() > -300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}
	while (!((FrontL.get_position() < -300*revolutions+5) && (FrontL.get_position() > -300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}





	while (!((BackR.get_position() < 300*revolutions+5) && (BackR.get_position() > 300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}

	while (!((MiddleR.get_position() < 300*revolutions+5) && (MiddleR.get_position() > 300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}
	while (!((FrontR.get_position() < 300*revolutions+5) && (FrontR.get_position() > 300*revolutions-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(10);
	}
	//*/
	
}

double average_encoders(){

    // pros::Motor BackL(11);
	// pros::Motor MiddleL(12);
	// pros::Motor FrontL(13);
	// pros::Motor BackR(20);
	// pros::Motor MiddleR(19);
	// pros::Motor FrontR(17);
    // fabs(MiddleL.get_position()) +
	//fabs(MiddleR.get_position()) +
	return (fabs(BackL.get_position()) +
            fabs(MiddleL.get_position()) +
            fabs(FrontL.get_position()) +
            fabs(BackR.get_position()) +
			fabs(MiddleR.get_position()) +
            fabs(FrontR.get_position()) ) / 6;
}    

void reset_encoders(){
	// pros::Motor BackL(11);
	// pros::Motor MiddleL(12);
	// pros::Motor FrontL(13);
	// pros::Motor BackR(20);
	// pros::Motor MiddleR(19);
	// pros::Motor FrontR(17);
    BackL.tare_position();
    MiddleL.tare_position();
    FrontL.tare_position();
    BackR.tare_position();
    MiddleR.tare_position();
    FrontR.tare_position();
}

void rotate(double rotateAngle){
	
	// Inertial.tare_rotation();

	double integral = 0 ;
	double derivative = 0 ;
	double error = 0;
	double kP=4.75;
	int iter = 0;
	double kI = 0;
	double kD=0.75;
	double lastError = 0;
	pros::c::gps_status_s_t status;
  	status = gps1.get_status();
	double targetAngle = rotateAngle + status.yaw;
	while( abs(status.yaw- targetAngle ) > 0.5 ){
		iter++;
		status = gps1.get_status();
		error = targetAngle - status.yaw;
		//calculate integral
		integral = integral + error * 0.02;
		//calculate derivative
		derivative = (error - lastError)/0.02;

		if(abs(error) == 0 ){
		integral = 0;
		}

		if(integral > 0.5){
		integral = 0.5;
		}

   		double resp = (error*kP + integral*kI + derivative*kD)*100;


		// FrontL.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
		// MiddleL.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
		// BackL.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
		// FrontR.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
		// MiddleR.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
		// BackR.spin(fwd, (-1)*resp, voltageUnits::millivolts); 
  	}
	
	// FrontL.spin(0); 
	// MiddleL.spin(0);
	// BackL.spin(0); 
	// FrontR.spin(0);
	// MiddleR.spin(0);
	// BackR.spin(0);
}

void move_encoder(double ticks){
  BackL.move_relative(-ticks, 100);
  MiddleL.move_relative(-ticks, 100);
  FrontR.move_relative(-ticks, 100);
  BackR.move_relative(ticks, 100);
  MiddleR.move_relative(ticks, 100);
  FrontR.move_relative(ticks, 100);
  while(average_encoders()<ticks){
    pros::delay(2);
  }
  FrontL.move_voltage(0); 
  MiddleL.move_voltage(0);
  BackL.move_voltage(0); 
  FrontR.move_voltage(0);
  MiddleR.move_voltage(0);
  BackR.move_voltage(0);
}