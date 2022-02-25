#include "main.h"
#include "daniel_auton.h"
#include "drive.h"
#include "algos.h"

double targetAngle;
double targetX;
double enablePID = true;

void daniel_auton (){
    pros::Task bill_nye(pid_loop_x);
    targetAngle = 0;
    targetX = 0.3;
    
}

int pid_loop_x (){
    pros::Motor BackR(11);
    pros::Motor MiddleR(12);
    pros::Motor FrontR(13);
    pros::Motor BackL(18);
    pros::Motor MiddleL(19);
    pros::Motor FrontL(20);
    pros::Imu Inertial(15);
    pros::c::gps_status_s_t status;

    double lat_integral = 0 ;
    double lat_deriv = 0 ;
    double lat_error = 0;
    double lat_last_error = 0;

    double ang_integral = 0 ;
    double ang_deriv = 0 ;
    double ang_error = 0;
    double ang_last_error = 0;
   
    double kP=0.2;
    int iter = 0;
    double kI = 0.045;
    double kD=0.035;
   
    while(enablePID){
      status = get_gps_heading();
      
      lat_error = targetX - status.x;
      //calculate lat_integral
      lat_integral = lat_integral + lat_error * 0.02;
      //calculate ang_derivative
      lat_deriv = (lat_deriv - lat_last_error)/0.02;
      // if(abs(error) == 0 ){ integral = 0; }
      // if(integral > 0.5){ integral = 0.5; }
      double lat_power = (lat_error*kP + lat_integral*kI + lat_integral*kD)*100;
      
      
      ang_error = targetAngle - status.yaw;
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      if(abs(ang_error) == 0 ){ ang_integral = 0; }
      if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*kP + ang_integral*kI + ang_deriv*kD)*100;

      move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;

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

int pid_loop_y (){
    pros::Motor BackR(11);
    pros::Motor MiddleR(12);
    pros::Motor FrontR(13);
    pros::Motor BackL(18);
    pros::Motor MiddleL(19);
    pros::Motor FrontL(20);
    pros::Imu Inertial(15);
    pros::c::gps_status_s_t status;

    double lat_integral = 0 ;
    double lat_deriv = 0 ;
    double lat_error = 0;
    double lat_last_error = 0;

    double ang_integral = 0 ;
    double ang_deriv = 0 ;
    double ang_error = 0;
    double ang_last_error = 0;
   
    double kP=0.2;
    int iter = 0;
    double kI = 0.045;
    double kD=0.035;
   
    while(enablePID){
      status = get_gps_heading();
      
      lat_error = targetX - status.y;
      //calculate lat_integral
      lat_integral = lat_integral + lat_error * 0.02;
      //calculate ang_derivative
      lat_deriv = (lat_deriv - lat_last_error)/0.02;
      // if(abs(error) == 0 ){ integral = 0; }
      // if(integral > 0.5){ integral = 0.5; }
      double lat_power = (lat_error*kP + lat_integral*kI + lat_integral*kD)*100;
      
      
      ang_error = targetAngle - status.yaw;
      //calculate ang_integral
      ang_integral = ang_integral + ang_error * 0.02;
      //calculate ang_derivative
      ang_deriv = (ang_error - ang_last_error)/0.02;
      if(abs(ang_error) == 0 ){ ang_integral = 0; }
      if(ang_integral > 0.5){ ang_integral = 0.5; }
      double ang_power = (ang_error*kP + ang_integral*kI + ang_deriv*kD)*100;

      move_base(lat_power + ang_power, lat_power - ang_power);

      ang_last_error = ang_error;
      lat_last_error = lat_error;

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

void travel2point(double t_pos [2], double speed){
  pros::Motor BackR(11);
	pros::Motor MiddleR(12);
	pros::Motor FrontR(13);
	pros::Motor BackL(18);
	pros::Motor MiddleL(19);
	pros::Motor FrontL(20);
	pros::Imu Inertial(15);
  pros::c::gps_status_s_t status;
  //Pos is in m, heading is neg ccw, pos cw in degrees
  double x_c_pos = 0;
  double y_c_pos = 0;
  double h_c = 0;
  while (abs(x_c_pos - t_pos[0]) > 0.3 || abs(y_c_pos - t_pos[1]) > 0.3){
    status = get_gps_heading();
    x_c_pos = status.x;
    y_c_pos = status.y;
    h_c = status.yaw * (3.1415/180);
    //h_c = get gps heading (IN RADIANS)
    double h_vec [2] = {sin(h_c * -1), cos(h_c * -1)};
    double ortho_vec [2] = {h_vec[1], -1*h_vec[0]};
    double y_prime [2] = { t_pos[0] - x_c_pos, t_pos[1] - y_c_pos};
    double t_dist = pow(y_prime[0] * y_prime[0]  + y_prime[1] * y_prime[1], 0.5);
    double r = t_dist / (2 * (y_prime[0] * ortho_vec[0] + y_prime[1] * ortho_vec[1]));
    
    //double smaller_angle  = acos((y_prime[0]*h_vec[0] + y_prime[1]*h_vec[1])/t_dist);
    //double dist_mag = 2 * smaller_angle*r;
    double prop_cmd [2] = { r + 0.3759/2 , r - 0.3759/2};
    double max_prop_mag = abs(prop_cmd[0]);
    if (abs(prop_cmd[1]) > max_prop_mag){
      max_prop_mag = abs(prop_cmd[1]);
    }
    double min_prop = prop_cmd[0];
    if (prop_cmd[1] < min_prop){
      min_prop = prop_cmd[1];
    }
    if (max_prop_mag == -1*min_prop){
      prop_cmd[0] = prop_cmd[0] * -1;
      prop_cmd[1] = prop_cmd[1] * -1;
    }
    double prop_mag = pow( pow(prop_cmd[0], 2) + pow(prop_cmd[1], 2), 0.5);
    prop_cmd[0] = prop_cmd[0] * speed / prop_mag;
    prop_cmd[1] = prop_cmd[1] * speed / prop_mag;

    FrontL.move_velocity((prop_cmd[0])*2); 
    MiddleL.move_velocity((prop_cmd[0])*2); 
    BackL.move_velocity((prop_cmd[0])*2); 
    FrontR.move_velocity((-1 * prop_cmd[1])*2);
    MiddleL.move_velocity((-1 * prop_cmd[1])*2);
    BackR.move_velocity((-1 * prop_cmd[1])*2);
  }
	FrontL.move_voltage(0); 
	MiddleL.move_voltage(0);
	BackL.move_voltage(0); 
	FrontR.move_voltage(0);
	MiddleR.move_voltage(0);
	BackR.move_voltage(0);
}

void turn2point(double target [2]){
  double x_c_pos = 0;
  double y_c_pos = 0;
  // rotate(getTheta(target));
  // pi_c t_pid;
  // while (abs(getTheta(target) > 0.1)){
  //   double resp = t_pid.update(getTheta(target));
  //   FrontL.spin(directionType::fwd, resp, velocityUnits::pct); 
  //   FrontR.spin(directionType::fwd, resp, velocityUnits::pct);
  //   BackL.spin(directionType::fwd, resp, velocityUnits::pct); 
  //   BackR.spin(directionType::fwd, resp, velocityUnits::pct);
  // }
  // FrontL.spin(directionType::fwd, 0, velocityUnits::pct); 
  // FrontR.spin(directionType::fwd, 0, velocityUnits::pct);
  // BackL.spin(directionType::fwd, 0, velocityUnits::pct); 
  // BackR.spin(directionType::fwd, 0, velocityUnits::pct);
}
