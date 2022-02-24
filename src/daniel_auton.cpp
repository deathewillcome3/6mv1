#include "main.h"
#include "daniel_auton.h"
#include "drive.h"
#include "algos.h"

void daniel_auton (){
    pros::Task bill_nye(pid_loop);
    rotate(90);
    
}

int pid_loop (){
    // while(enablePID){

    //     pros::Task::delay(20);
    // }
    return 1;
}

