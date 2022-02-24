#ifndef getTheta_h
#define getTheta_h
double getTheta(double target [2]);
#endif

#ifndef get_gps_heading_h
#define get_gps_heading_h
pros::c::gps_status_s_t get_gps_heading();
#endif

#ifndef path_extrapolation_h
#define path_extrapolation_h
std::vector<double> path_extrapolation(std::vector<double> path, double spacing);
#endif
