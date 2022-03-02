#include "main.h"
#include "drive.h"
#include "algos.h"
#include "globals.h"


void move_base(double left, double right){
    BackL = -left;
    MiddleL = -left;
    FrontL = left;
    BackR = -right;
    MiddleR = -right;
    FrontR = right;
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
            
            fabs(FrontL.get_position()) +
            fabs(BackR.get_position()) +
            fabs(FrontR.get_position()) ) / 4;
}    

void reset_encoders(){
	// pros::Motor BackL(11);
	// pros::Motor MiddleL(12);
	// pros::Motor FrontL(13);
	// pros::Motor BackR(20);
	// pros::Motor MiddleR(19);
	// pros::Motor FrontR(17);
    BackL.tare_position();
    // MiddleL.tare_position();
    FrontL.tare_position();
    BackR.tare_position();
    // MiddleR.tare_position();
    FrontR.tare_position();
}

void rotate(double rotateAngle){
	pros::c::gps_status_s_t status = get_gps_heading();
	// Inertial.tare_rotation();
	double integral = 0 ;
	double derivative = 0 ;
	double error = 0;
	double kP=0.2;
	int iter = 0;
	double kI = 0.045;
	double kD=0.035;
	double lastError = 0;
	while( abs(status.yaw- rotateAngle ) > 0.5 ){
		iter++;
		if(iter >= 50) {
		break;
		}
		error = rotateAngle - status.yaw;
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


		FrontL.move_voltage(resp); 
		MiddleL.move_voltage((-1)*resp);
		BackL.move_voltage((-1)*resp); 
		FrontR.move_voltage((-1)*resp);
		MiddleR.move_voltage(resp);
		BackR.move_voltage(resp);

		lastError = error;
		pros::delay(20);
  	}
	FrontL.move_voltage(0); 
	MiddleL.move_voltage(0);
	BackL.move_voltage(0); 
	FrontR.move_voltage(0);
	MiddleR.move_voltage(0);
	BackR.move_voltage(0);
}

