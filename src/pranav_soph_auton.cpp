#include "main.h"
#include "pranav_soph_auton.h"
#include "drive.h"
#include "algos.h"


//amt of time it takes to move 1 tile (not correct right now)
double unit = 1;
double arm_degrees = 1;

//I think this is gonna be a left auton
void pranav_soph_auton (){

    //Initializing? - copied from the main "opcontrol" function

    //controller unecessary 
    //pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor BackL(4);
	pros::Motor MiddleL(6);
	pros::Motor FrontL(5);
	pros::Motor BackR(1);
	pros::Motor MiddleR(2);
	pros::Motor FrontR(3);
	pros::Motor Arm1(7);
	//pros::Motor Arm2(5);
	pros::Motor Elev(8); //apprently the rings motor according to danny lmao
	pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');

    //initiliaze the pistons as well ig

    
    left_auton();

}



void left_auton (){
    //Lud's auton 6
    //starts w base move (0,0)?

    //red w back arm
    //pick up left neutral w front arm?

    //**NOTE FROM PRANAV**
    //**THE MEASUREMENTS ARE LIKE REALLY REALLY REALLY REALLY SUS**
    //**SO LIKE THEY DEF DEF NEED TO BE FINE TUNED**
    //**ANGLE MEASUREMENTS + DRIVE LENGTHS SHOULD BE CHECKED**
    //**CUZ IDK WHAT IM DOING LMFAO**
    //**SO THE CURRENT MAX ARM ANGLE IS 60 AND IM ESTIMATING AN ANGLE OF 35 ON THE BALANCES**

    //initializing cuz this is dumb
    pros::Motor BackL(4);
	pros::Motor MiddleL(6);
	pros::Motor FrontL(5);
	pros::Motor BackR(1);
	pros::Motor MiddleR(2);
	pros::Motor FrontR(3);
	pros::Motor Arm1(7);
	//pros::Motor Arm2(5);
	pros::Motor Elev(8); //apprently the rings motor according to danny lmao
	pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');

    //initiliaze the pistons as well ig
    

    //resetting to 0 iDK mAN
    move_base(0,0); //imma just assume i s+-hould keep this
    Arm1.move(0);


    //unclamped
    pistonF.set_value(false);
    pistonB1.set_value(true);
    pistonB2.set_value(true);
    //front pistons: False = unclamped, true = clamped
    //back pistons: true = unclamped, false = clamped
    

    /*
    //Pranav's code :D
    //Starting with back clamp facing towards the blue tower
    
    pistonB1.set_value(true);
    pistonB2.set_value(true);

    //turn and go try and fight that neutral tower dude
    move_backward(1);
    rotate(90);
    move_forward(2);
    pistonF.set_value(true);


    //now move to place cuz we chadded
    //this def wont work tho #nofaith
    Arm1.move(60);
    rotate(45);
    move_forward(1.12);
    Arm1.move(-35);
    pros::delay(1);
    pistonF.set_value(false);
    move_backward(0.12);
    pistonB1.set_value(false);
    pistonB2.set_value(false);
    rotate(180);
    Arm1.move(-25);
    PistonF.set_value(true);
    Arm1.move(60);
    rotate(180);
    move_forward(0.12);
    Arm1.move(-35);
    pros::delay(1);
    
    //so assuming that didnt completely go to shit, we grab the red (line) and try to not die
    rotate(-135);
    move_forward(1.8);
    Arm1.move(-25);
    move_foward(0.2);
    pistonF.set_value(true);

    //hopefully it all didnt go to crap there so we try to place it on the balance
    rotate(-45);
    Arm1.move(60);
    move_forward(4.3);
    Arm1.move(-35);
    pros::delay(1);
    pistonF.set_value(false);

    //now for blue (line) and neutral right
    rotate(-45);
    Arm1.move(-25);
    move_forward(2);
    pistonF.set_value(true);
    Arm1.move(60);
    rotate(90);
    move_foward(1.5);
    pistonB1.set_value(true);
    pistonB2.set_value(true);

    //if somehow, against all odds, we're still in this... we place the blue and push the neutral
    rotate(-45);
    move_forward(2.82);
    Arm1.move(-35);
    pros::delay(1);
    pistonF.set_value(false);
    rotate(-45);
    move_backward(3);
    pistonB1.set_value(false);
    pistonB1.set_value(false);
    //*/




    //Sop's code attempt
    //varun is here too
    //clamping blue tower 
    move_backward(1);
    clampB();
    //goes off the ramp
    //might be unecessary depending on what happens when we test
    move_forward(0.5);


    //Turning right without rotating in place
    //rotating in place would cause the mobile base to hit
    move_base(100, 0);
    //front clamp faces yellow tower
    //and we just pray it lines up even though it won't
    //I think we just have to adjust it when we test HNNGGHHH

    move_forward(3.5);
    clampF();
    armUp();
    
    //want to stack on balance
    //these are literally random values
    rotate(45);
    move_forward(2.5);


}



/*void right_auton (){

}

void neutral_auton (){

}*/




void armUp(){
    pros::Motor Arm1(7);
    Arm1.move(120*arm_degrees);
}

void armDown(){
    pros::Motor Arm1(7);
    Arm1.move(-120*arm_degrees);
}



//HOW DIVAD SAYS TO CLAMP
/*
Arm up

drive up to balance
go down a tiny bit
unclamp
***go back a tiny bit
go back up
go backwards
*/



void clampF(){
    pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');
    pistonF.set_value(true);
}
void clampB(){
    pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');
    pistonB1.set_value(false);
    pistonB2.set_value(false);
}

void unclampF(){
    pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');
    pistonF.set_value(false);
}
void unclampB(){
    pros::ADIDigitalOut pistonF('A');
	pros::ADIDigitalOut pistonB1('B');
    pros::ADIDigitalOut pistonB2('C');
    pistonB1.set_value(true);
    pistonB2.set_value(true);
}

//movement helper functions
//that don't do anything
void move_forward(double tile){
    double time = tile*unit*1000;
    move_base(100, 100);
    pros::delay(time);
    move_base(0,0);
}

void move_backward(double tile){
    double time = tile*unit*1000;
    move_base(-100, -100);
    pros::delay(time);
    move_base(0,0);
}


void turn_left(){

}

void turn_right(){

}



//*/

//From looking at Luds old code, "base_move" seems like an important function but i have no idea what it does :D
//

/*
void base_move(double main_response, double side_response){
    FrontL.spin(directionType::fwd, side_response + main_response, velocityUnits::pct);
    FrontR.spin(directionType::fwd, side_response - main_response, velocityUnits::pct);
    BackL.spin(directionType::fwd, side_response + main_response, velocityUnits::pct);
    BackR.spin(directionType::fwd, side_response - main_response, velocityUnits::pct);

    FrontL.setVelocity(side_response + main_response, vex::percentUnits::pct);
    FrontR.setVelocity(side_response - main_response, vex::percentUnits::pct);
    BackL.setVelocity(side_response + main_response, vex::percentUnits::pct);
    BackR.setVelocity(side_response - main_response, vex::percentUnits::pct);
}//*/



/*

viveks pro advice:
double unit - how many seconds it takes to move 1 tile
double degree - input it takes to turn 90 degrees, and the divide by 90

helper functions:
moving forward
move backwards
turn left
turn right



*/