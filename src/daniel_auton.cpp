#include "main.h"
#include "daniel_auton.h"
#include "drive.h"
#include "algos.h"
#include "globals.h"

double targetAngle;
double targetX;
double enablePID = true;

void daniel_auton (){
  Inertial.reset();
  // FrontL.move(-63);
  // FrontR.move(63);
  // BackL.move(-63);
  // BackR.move(63);
  // pros::delay(200);
  // FrontL.move(0);
  // FrontR.move(0);
  // BackL.move(0);
  // BackR.move(0);
  reset_encoders();
  pros::Task billnye(pid_loop_x);
  targetAngle = 0;
  targetX = 300;
  pros::delay(2000);
  // double t_pos[2] = {0, 0};
  // travel2point(t_pos, 80);
  // rotate(-90);

}

int pid_loop_gps (){
    pros::c::gps_status_s_t status;

    double lat_integral = 0 ;
    double lat_deriv = 0 ;
    double lat_error = 0;
    double lat_last_error = 0;

    double ang_integral = 0;
    double ang_deriv = 0 ;
    double ang_error = 0;
    double ang_last_error = 0;
   
    double kP = 1.5;
    int iter = 0;
    double kI = 0;
    double kD= 1.5;
   
   
    while(enablePID){
      status = get_gps_heading();

      pros::screen::print(TEXT_MEDIUM, 1, "Encoder Position: %3f", status.yaw);

      lat_error= targetX - status.x;
      //calculate lat_integral
      lat_integral = lat_integral + lat_error * 0.02;
      //calculate ang_derivative
      lat_deriv = (lat_deriv - lat_last_error)/0.02;
      // if(abs(error) == 0 ){ integral = 0; }
      // if(integral > 0.5){ integral = 0.5; }
      double lat_power = (lat_error*kP + lat_integral*kI + lat_integral*kD);
      
      ang_error = targetAngle - status.yaw;
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      // if(abs(ang_error) == 0 ){ ang_integral = 0; }
      // if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*kP  + ang_deriv*kD);
      pros::screen::print(TEXT_MEDIUM, 2, "Angular Position: %3f", ang_error);
      pros::screen::print(TEXT_MEDIUM, 3, "Angular Power: %3f", ang_power);
      
      // move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;

      FrontL.move(-lat_power - ang_power);
      FrontR.move(lat_power - ang_power);
      BackL.move(-lat_power - ang_power);
      BackR.move(lat_power - ang_power);
      pros::delay(20);

  }

  FrontL.move_voltage(0); 
  MiddleL.move_voltage(0);
  BackL.move_voltage(0); 
  FrontR.move_voltage(0);
  MiddleR.move_voltage(0);
  BackR.move_voltage(0);

  return 1;
}

int pid_loop_x (){

    pros::c::gps_status_s_t status;

    double lat_integral = 0 ;
    double lat_deriv = 0 ;
    double lat_error = 0;
    double lat_last_error = 0;

    double ang_integral = 0;
    double ang_deriv = 0 ;
    double ang_error = 0;
    double ang_last_error = 0;
   
    double kP = 1.5;
    int iter = 0;
    double kI = 0;
    double kD= 1.5;
   
   
    while(enablePID){
      status = gps1.get_status();
      pros::screen::print(TEXT_MEDIUM, 1, "Encoder Position: %3f", status.yaw);

      lat_error= targetX - average_encoders();
      //calculate lat_integral
      lat_integral = lat_integral + lat_error * 0.02;
      //calculate ang_derivative
      lat_deriv = (lat_deriv - lat_last_error)/0.02;
      // if(abs(error) == 0 ){ integral = 0; }
      // if(integral > 0.5){ integral = 0.5; }
      double lat_power = (lat_error*kP + lat_integral*kI + lat_integral*kD);
      
      ang_error = targetAngle - status.yaw;
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      // if(abs(ang_error) == 0 ){ ang_integral = 0; }
      // if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*kP  + ang_deriv*kD);
      pros::screen::print(TEXT_MEDIUM, 2, "Angular Position: %3f", ang_error);
      pros::screen::print(TEXT_MEDIUM, 3, "Angular Power: %3f", ang_power);
      
      // move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;

      FrontL.move(-lat_power - ang_power);
      FrontR.move(lat_power - ang_power);
      BackL.move(-lat_power - ang_power);
      BackR.move(lat_power - ang_power);
      pros::delay(20);

  }

  FrontL.move_voltage(0); 
  MiddleL.move_voltage(0);
  BackL.move_voltage(0); 
  FrontR.move_voltage(0);
  MiddleR.move_voltage(0);
  BackR.move_voltage(0);

  return 1;
}

