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
