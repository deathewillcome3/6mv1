#include "main.h"
#include "algos.h"

void travel2point(double t_pos [2], double speed){
  pros::Motor BackR(11);
	pros::Motor MiddleR(12);
	pros::Motor FrontR(13);
	pros::Motor BackL(18);
	pros::Motor MiddleL(19);
	pros::Motor FrontL(20);
	pros::Imu Inertial(15);
  // pros::Gps gps1(24);
  // pros::Gps gps2(23);
  // pros::gps_status_s_t status;
  //Pos is in m, heading is neg ccw, pos cw in degrees
  double x_c_pos = 0;
  double y_c_pos = 0;
  double h_c = 0;
  while (abs(x_c_pos - t_pos[0]) > 0.3 || abs(y_c_pos - t_pos[1]) > 0.3){
    // status1 = gps1.get_status();
    // status2 = gps2.get_status();
    // status1.yaw;
    //x_c_pos = get gps pos
    //y_c_pos = get gps pos
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

double getTheta(double target [2]){
  double pos [2] = {0,0};
  double h_c = 0;
  // pos[0] from gps
  // pos[1] from gps
  // h_c from gps (heading)
  double point_vec [2] = {target[0] - pos[0], target[1] - pos[1]};
  double pv_mag = pow( pow(point_vec[0], 2) + pow(point_vec[1], 2), 0.5);
  point_vec[0] = point_vec[0] / pv_mag;
  point_vec[1] = point_vec[1] / pv_mag;
  double h_vec [2] = {sin(h_c * -1), cos(h_c * -1)}; 
  double turn_angle = acos(h_vec[0] * point_vec[0] + h_vec[1] * point_vec[1]);
  double new_head [2] = { -1 * h_vec[1], h_vec[0]};
  double side = new_head[0] * point_vec[0] + new_head[1] * point_vec[1];
  double turn_ = 0;
  if (side > 0){
    turn_ = -1 * turn_angle;
  } else{
    turn_ = 1 * turn_angle;
  }
  return turn_;
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

