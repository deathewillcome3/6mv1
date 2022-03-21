#include "main.h"
#include "algos.h"
#include "globals.h"
#include <vector>
#include <cmath>

double getTheta(double target [2]){
  double pos [2] = {0,0};
  double h_c = 0;
  pros::c::gps_status_s_t gps_heading;
  gps_heading = gps1.get_status();
  // pos[0] from gps
  pos[0] = gps_heading.x;
  // pos[1] from gps
  pos[1] = gps_heading.y;
  // h_c from gps (heading)
  h_c = ((gps_heading.yaw)*3.1415)/180;
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

std::vector<std::vector<double>> path_extrapolation(std::vector<std::vector<double>> path, double spacing){
  std::vector<std::vector<double>> full_path;
  full_path.reserve(30);
  for(int i=0; i<path.size(); i++){
    if(i+1 == path.size()){
      full_path.push_back(path[i]);
    }
    else{
      std::vector<double> vect;
      vect[0] = path[i+1][0] - path[i][0];
      vect[1] = path[i+1][1] - path[i][1];
      double vect_mag = pow(pow(vect[0], 2) + pow(vect[1], 2), 0.5);
      double max_points = std::ceil(vect_mag / spacing);
      vect[0] = vect[0] / vect_mag;
      vect[1] = vect[1] / vect_mag;
      for (i=0; i < max_points; i++){
        std::vector<double> coord;
        coord [0] = path[i][0] + vect[0]*i;
        coord [1] = path[i][1] + vect[1]*i;
        full_path.push_back(coord);
      }

    }
  }
  return path;
}

pros::c::gps_status_s_t get_gps_heading(){
  // 6.5 inches from the center X Left GPS
  // 1.5 inches from the center Y left GPS 
  // 1.5 inches from the center Y Right GPS
  // 7.5 inches from the center X Right GPS 
  pros::c::gps_status_s_t status;
  pros::c::gps_status_s_t status1;
  status = gps1.get_status();
  status1 = gps2.get_status();
  if(status.x == 0 || status.y == 0 || status.yaw == 0 ){
    return status1;
  }
  else if (status1.x == 0 || status1.y == 0 || status1.yaw == 0 ){
    return status;
  }
  return status;
}

void travel2point(double t_pos [2], double speed){
  //Pos is in m, heading is neg ccw, pos cw in degrees
  pros::c::gps_status_s_t gps_heading;
  gps_heading = gps1.get_status();
  double pos [2] = {0,0};
  double x_c_pos = gps_heading.x;
  double y_c_pos = gps_heading.y;
  double h_c = 0;

  // pros::screen::print(TEXT_MEDIUM, 2, "heading: %3f", gps_heading.yaw);
  //abs(x_c_pos - t_pos[0]) > 0.3 && abs(y_c_pos - t_pos[1]) > 0.3
  while (abs(x_c_pos - t_pos[0]) > 0.05 && abs(y_c_pos - t_pos[1]) > 0.05){
    gps_heading = gps1.get_status();
    // pos[0] from gps
    x_c_pos = gps_heading.x;
    // pos[1] from gps
    y_c_pos = gps_heading.y;
    // h_c from gps (heading)
    h_c = ((gps_heading.yaw+90)*3.1415)/180;
    double h_vec [2] = {sin(h_c * -1), cos(h_c * -1)};
    double ortho_vec [2] = {h_vec[1], -1*h_vec[0]};
    double y_prime [2] = { t_pos[0] - x_c_pos, t_pos[1] - y_c_pos};
    double t_dist = pow(y_prime[0] * y_prime[0]  + y_prime[1] * y_prime[1], 0.5);
    double r = t_dist / (2 * (y_prime[0] * ortho_vec[0] + y_prime[1] * ortho_vec[1]));
    
    //double smaller_angle  = acos((y_prime[0]*h_vec[0] + y_prime[1]*h_vec[1])/t_dist);
    //double dist_mag = 2 * smaller_angle*r;
    //0.3759
    double prop_cmd [2] = { r - 0.1/2 , r + 0.1/2};
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

    FrontL.move_velocity((-1 * prop_cmd[0])*6); 
    MiddleL.move_velocity((-1 * prop_cmd[0])*6); 
    BackL.move_velocity((-1 * prop_cmd[0])*6); 
    FrontR.move_velocity((1 * prop_cmd[1])*6); 
    MiddleR.move_velocity((1 * prop_cmd[1])*6); 
    BackR.move_velocity((1 * prop_cmd[1])*6); 
    // pros::screen::print(TEXT_MEDIUM, 1, "R: %3f", r);
    // pros::screen::print(TEXT_MEDIUM, 2, "R: %3f", prop_cmd[0]);
    // pros::screen::print(TEXT_MEDIUM, 2, "R: %3f", prop_cmd[1]);

  }
  FrontL.move_velocity(0); 
  MiddleL.move_velocity(0); 
  BackL.move_velocity(0); 
  FrontR.move_velocity(0); 
  MiddleR.move_velocity(0); 
  BackR.move_velocity(0); 
}

// void turn2point(double target [2]){
//   double x_c_pos = 0;
//   double y_c_pos = 0;
//   pi_c t_pid;
//   while (abs(getTheta(target) > 0.1)){
//     double resp = t_pid.update(getTheta(target));
//     FrontL.move_velocity(-1 * resp)*6); 
//     MiddleL.move_velocity(-1 * resp)*6); 
//     BackL.move_velocity(-1 * resp)*6); 
//     FrontR.move_velocity(-1 * resp)*6); 
//     MiddleR.move_velocity(-1 * resp)*6); 
//     BackR.move_velocity(-1 * resp)*6); 
//   }
//   FrontL.move_velocity(0); 
//   MiddleL.move_velocity(0); 
//   BackL.move_velocity(0); 
//   FrontR.move_velocity(0); 
//   MiddleR.move_velocity(0); 
//   BackR.move_velocity(0); 
// }

// Returns a distance based on the current robot position
double current_distance(std::vector<double> target){
  pros::c::gps_status_s_t status;
  status = gps1.get_status();
  return pow( pow(target[0]-status.x, 2) + pow(target[1]-status.y, 2), 0.5);
}

//Returns a distance based on two points
double distance(std::vector<std::vector<double>> points){
  return pow( pow(points[0][0]-points[1][0], 2) + pow(points[0][1]-points[1][1], 2), 0.5);
}

//Returns the arc length of three intersecting points
double curvature(std::vector<std::vector<double>> points){
  double x1 = points[0][0] + 0.001;
  double y1 = points[0][1];
  double x2 = points[1][0];
  double y2 = points[1][1];
  double x3 = points[2][0];
  double y3 = points[2][1];
  double k1 = 0.5 * (pow(points[0][0], 2) + pow(points[0][1], 2) - pow(points[1][0], 2) - pow(points[1][1],2)) /
    (points[0][0]-points[1][0]);
  double k2 = (y1-y2) / (x1-x2);
  double b = 0.5 * (pow(x2, 2) - 2*x2*k1 + pow(y2,2) - pow(x3,2) + 2*x3*k1 - pow(y3, 2)) / 
    (x3*k2 - y3 + y2 - x2*k2);
  double a = k1 - k2*b;
  // double r = pow((pow(x1-a, 2) + pow(y1-b,2), 0.5));
  return 1/a;
}


double get_angle(std::vector<double> target, bool left){
  pros::c::gps_status_s_t status;
  status = gps1.get_status();
  double distancex, distancey;
  if (status.y > 0 && target[1] > 0){
    distancey= abs(status.y-target[1]);
  }
  if (status.y < 0 && target[1] < 0 ){
    distancey = abs(abs(status.y) - abs(target[1]));
  }
  if (status.y > 0 && target[1] < 0){
    distancey = status.y-target[1];
  }
   if (status.y < 0 && target[1] > 0){
    distancey = target[1]-status.y;
  }

  if (status.x > 0 && target[0] > 0){
    distancex= abs(status.x-target[0]);
  }
  if (status.x < 0 && target[0] < 0 ){
    distancex = abs(abs(status.x) - abs(target[0]));
  }
  if (status.x > 0 && target[0] < 0){
    distancex = status.x-target[0];
  }
   if (status.x < 0 && target[0] > 0){
    distancex = target[0]-status.x;
  }
  
  double angle = (atan2(distancey, distancex))/3.1415926 * 180;

   if (left == true){
     angle = angle*-1;
  }
  return angle;

}
