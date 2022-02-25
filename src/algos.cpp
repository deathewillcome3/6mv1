#include "main.h"
#include "algos.h"
#include <vector>
#include <cmath>

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

// std::vector<double [2]> path_extrapolation(std::vector<double [2]> path, double spacing){
//   std::vector<double [2]> full_path;
//   full_path.reserve(30);
//   for(int i=0; i<path.size(); i++){
//     if(i+1 == path.size()){
//       full_path.push_back(path[i]);
//     }
//     else{
//       double vect [2];
//       vect[0] = path[i+1][0] - path[i][0];
//       vect[1] = path[i+1][1] - path[i][1];
//       double vect_mag = pow(pow(vect[0], 2) + pow(vect[1], 2), 0.5);
//       double max_points = std::ceil(vect_mag / spacing);
//       vect[0] = vect[0] / vect_mag;
//       vect[1] = vect[1] / vect_mag;
//       for (i=0; i < max_points; i++){
//         double coord [2];
//         coord [0] = path[i][0] + vect[0]*i;
//         coord [1] = path[i][1] + vect[1]*i;
//         full_path.push_back(coord);
//       }

//     }
//   }
//   return path;
// }

pros::c::gps_status_s_t get_gps_heading(){
  // 6.5 inches from the center X Left GPS
  // 1.5 inches from the center Y left GPS 
  // 1.5 inches from the center Y Right GPS
  // 7.5 inches from the center X Right GPS 
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

