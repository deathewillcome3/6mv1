#include "main.h"
#include "algos.h"
#include <vector>

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

std::vector<double> path_extrapolation(std::vector<double> path, double spacing){
  for(int i=0; i<path.size(); i++){
    if(i+1 == path.size()){
      // output.append(path[i])
    }
    else{
      
    }
  }
  return path;
}

pros::c::gps_status_s_t get_gps_heading(){
  pros::Gps gps(24);
  pros::Gps gps1(23);
  pros::c::gps_status_s_t status;
  pros::c::gps_status_s_t status1;
  status = gps.get_status();
  status1 = gps1.get_status();
  if(status.x == 0 || status.y == 0 || status.yaw == 0 ){
    return status1;
  }
  else if (status1.x == 0 || status1.y == 0 || status1.yaw == 0 ){
    return status;
  }
  return status;
}

