#ifndef getTheta_h
#define getTheta_h
double getTheta(double target [2]);
#endif

#ifndef get_gps_heading_h
#define get_gps_heading_h
pros::c::gps_status_s_t get_gps_heading();
#endif

#ifndef travel2point_h
#define travel2point_h
void travel2point(double t_pos [2], double speed);
#endif

#ifndef path_extrapolation_h
#define path_extrapolation_h
std::vector<std::vector<double>> path_extrapolation(std::vector<std::vector<double>> path, double spacing);
#endif

#ifndef current_distance_h
#define current_distance_h
double current_distance(std::vector<double> target);
#endif 

#ifndef distance_h
#define distance_h
double distance(std::vector<std::vector<double>> points);
#endif

#ifndef curvature_h
#define curvature_h
double curvature(std::vector<std::vector<double>> points);
#endif

#ifndef getAngle_h
#define getAngle_h
double get_angle(std::vector<double> target, bool left);
#endif