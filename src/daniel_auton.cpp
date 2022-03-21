#include "main.h"
#include "daniel_auton.h"
#include "drive.h"
#include "algos.h"
#include "globals.h"

double target_angle;
double targetX;
std::vector<double> targetP;
double enablePID = true;
bool rotateOnly=false;

void daniel_auton (){
  pros::c::gps_status_s_t status;
  Inertial.reset();
  reset_encoders();
  status = gps1.get_status();
  double angle [2] = {-1.079974,   -1.066786};
  //getTheta(angle)*180
  //get_angle(angle)
  // pros::screen::print(TEXT_MEDIUM, 1, "targetPos: %3f",angle[0]);
  // pros::screen::print(TEXT_MEDIUM, 2, "targetPos: %3f", angle[1]);
  // pros::screen::print(TEXT_MEDIUM, 3, "currentX: %3f", status.x);
  // pros::screen::print(TEXT_MEDIUM, 4, "currentY: %3f", status.y);
  // pros::screen::print(TEXT_MEDIUM, 5, "get_angle: %3f", get_angle(angle, true));
  // pros::screen::print(TEXT_MEDIUM, 6, "getTheta: %3f", getTheta(angle));
  //  pros::screen::print(TEXT_MEDIUM, 7, "gps: %3f", status.yaw);
  // rotate (get_angle(angle));
  // pros::Task 

  //Gets out of intial deadzone
  move_encoder(-800);
  move_encoder(300);
  
  //Defines point and rotates to defined point
  targetP [0] = -0.175305;
  targetP [1] = -1.013118;
  rotateOnly = true;
  target_angle = get_angle(target, false);
  pros::Task billnye(pid_loop_gps);
  // while(abs(status.yaw-get_angle(angle,true))> 5){
  //   pros::delay(20);
  // }
  pros::delay(1000);
  //Moves to defined point
  rotateOnly = false;
  pros::delay(1000);

  //Point 2 Same code 
  rotateOnly = true;
  targetP [0] = 0.7993565;
  targetP [1] = -0.273173;
  target_angle = get_angle(target, true);
  // while(abs(status.yaw-get_angle(angle,true))> 5){
  //   pros::delay(20);
  // }
  pros::delay(1000);
  rotateOnly = false;
  // pros::delay(1000);
  // rotateOnly = true;
  // targetP [0] = 0.9319615;
  // targetP [1] = 1.3449086;
  // target_angle = get_angle(targetP, true);
  // pros::delay(1000);
  // rotateOnly = false;
  // targetX = 1800;
  // double t_pos[2] = {0, 0};
  // rotate(90);
  // double target[2] = {0, -1};
  // double target[2] = { 0.7993565, -0.273173};
  // // rotate(getTheta(target));
  // travel2point(target, 100);
  // double target1[2] = { 0.9319615, 1.3449086};
  // pros::screen::print(TEXT_MEDIUM, 1, "Testing: %3f", getTheta(target1));
  // rotate(getTheta(target1));
  // FrontL.move_velocity(0); 
  // MiddleL.move_velocity(0); 
  // BackL.move_velocity(0); 
  // FrontR.move_velocity(0); 
  // MiddleR.move_velocity(0); 
  // BackR.move_velocity(0); 
  // pros::delay(1000);
  // rotate(0);
  // pros::delay(1000);
  // rotate(-45);
  // pros::c::gps_status_s_t gps_heading;
  // gps_heading = gps1.get_status();
  // pros::screen::print(TEXT_MEDIUM, 2, "Yaw: %3f", gps_heading.yaw);
  // travel2point(target1, 80);
  // target[0] = 0.9319615;
  // target[1] = 1.3449086;
  // rotate(getTheta(target));
  // travel2point(target, 80);
}

void pid_move_point(std::vector<double> target, double delay){
  rotateOnly = true;
  target_angle = get_angle(target, false);
  pros::Task billnye(pid_loop_gps);
  // while(abs(status.yaw-get_angle(angle,true))> 5){
  //   pros::delay(20);
  // }
  pros::delay(1000);
  rotateOnly = false;
  pros::delay(delay);
}

int pid_loop_gps (){
    pros::c::gps_status_s_t status;

    double lat_integral = 0 ;
    double lat_deriv = 0 ;
    double lat_error = 0;
    double lat_last_error = 0;

    double lat_kP = 1.5;
    int lat_iter = 0;
    double lat_kI = 0;
    double lat_kD= 0.02;

    double ang_integral = 0;
    double ang_deriv = 0 ;
    double ang_error = 0;
    double ang_last_error = 0;
   
    double ang_kP = 3.5;
    int ang_iter = 0;
    double ang_kI = 0;
    double ang_kD= 0;
   
   
    while(enablePID){
      status = gps1.get_status();
      if(current_distance(targetP)> 0.15){
        lat_error = current_distance(targetP)*100;
      }
      else{
        lat_error = 0;
      }

      //calculate lat_integral
      lat_integral = lat_integral + lat_error * 0.02;
      //calculate ang_derivative
      lat_deriv = (lat_deriv - lat_last_error)/0.02;
      // if(abs(error) == 0 ){ integral = 0; }
      // if(integral > 0.5){ integral = 0.5; }
      double lat_power = (lat_error*lat_kP + lat_integral*lat_kI + lat_integral*lat_kD);
      if(abs(target_angle - Inertial.get_heading()) > 5){
        ang_error = target_angle - Inertial.get_heading();
      }
      else{
        ang_error = 0;
      }
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      // if(abs(ang_error) == 0 ){ ang_integral = 0; }
      // if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*ang_kP  + ang_deriv*ang_kD);
      // pros::screen::print(TEXT_MEDIUM, 2, "error: %3f", lat_error);
      // pros::screen::print(TEXT_MEDIUM, 3, "Angular Position: %3f", ang_error);
      // pros::screen::print(TEXT_MEDIUM, 4, "Angular Power: %3f", ang_power);
      // pros::screen::print(TEXT_MEDIUM, 5, "Lat Power: %3f", lat_power);
      
      // move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;

      if(rotateOnly) lat_power = 0;

      FrontL.move(-lat_power - ang_power);
      MiddleL.move(-lat_power - ang_power);
      BackL.move(-lat_power - ang_power);
      FrontR.move(lat_power - ang_power);
      MiddleR.move(lat_power - ang_power);
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
    double kD= 0.5;
   
   
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
      
      ang_error = target_angle - status.yaw;
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      // if(abs(ang_error) == 0 { ang_integral = 0; }
      // if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*kP  + ang_deriv*kD);
      pros::screen::print(TEXT_MEDIUM, 2, "error: %3f", lat_error);
      pros::screen::print(TEXT_MEDIUM, 3, "Angular Position: %3f", ang_error);
      pros::screen::print(TEXT_MEDIUM, 4, "Angular Power: %3f", ang_power);
      pros::screen::print(TEXT_MEDIUM, 5, "Lat Power: %3f", lat_power);
      
      // move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;
      ang_power = 0;

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

