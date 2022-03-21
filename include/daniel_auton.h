#ifndef daniel_auton_h
#define daniel_auton_h
void daniel_auton();
#endif

#ifndef pid_loop_h
#define pid_loop_h
int pid_loop_x();
#endif

#ifndef pid_loop_gps_h
#define pid_loop_gps_h
int pid_loop_gps();
#endif

#ifndef pid_loop_h_1
#define pid_loop_h_1
int pid_loop_y();
#endif


#ifndef turn2point_h
#define turn2point_H
void turn2point(double target [2]);
#endif

#ifndef pid_move_point_h
#define pid_move_point_h
void pid_move_point(std::vector<double> target, double delay);
#endif

// FrontL               motor         8               
// FrontR               motor         6               
// BackL                motor         4               
// BackR                motor         13              
// Ring                 motor         11              
// FrontArm             motor         15              
// Gyro                 inertial      5               
// Clamp                motor         19   
