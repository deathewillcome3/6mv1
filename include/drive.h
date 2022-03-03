#ifndef move_base_h
#define move_base_h
void move_base(double left, double right);
#endif

#ifndef move_good_h
#define move_good_h
void move_good(double rev);
#endif

#ifndef reset_encoders_h
#define reset_encoders_h
void reset_encoders();
#endif

#ifndef average_encoders_h
#define average_encoders_h
double average_encoders();
#endif

#ifndef translate_h
#define translate_h
void translate();
#endif

#ifndef rotate_h
#define rotate_h
void rotate(double rotateAngle);
#endif