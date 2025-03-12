#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
#include <DCMotor.h>    //DC Motor Movement functions
#include <StepperMotor.h> //Stepper Motor Movement function

void beacon_positioning(){ //Functioning (Hard-coded)
    move_function(24, 'B');
    usleep(450000);
    move_function(50, 'S');
    sleep(1);
    move_function(44, 'R');
    usleep(1000000);
    move_function(50, 'S');
    sleep(1);
    move_function(15, 'F');
    usleep(4000000);
    move_function(50, 'S');
    sleep(2);
    move_function(46, 'R');
    usleep(1100000);
    move_function(50, 'S');
    sleep(1);
    move_function(19, 'F');
    usleep(1200000);            //may need slight adjusting
    move_function(50, 'S');
    sleep(1);
}
void cube_pickups(){ //In-progress
    move_function(20, 'B');
    usleep(900000);
    move_function(50, 'S');
    sleep(1);
    move_function(54, 'L');
    usleep(11000000);
    move_function(50, 'S');
    sleep(1);
    move_function(20, 'B');
    usleep(3300000);
    move_function(50, 'S');
    sleep(1);

}
void lawnmower_path(){ //In progress
        //foward
        move_function(45, 1);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //rotate left
        move_function(30, -2);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //foward
        move_function(20, 1);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //rotate left
        move_function(30, -2);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1); 
}
void lawnmower_path_cave(){   //Not Started
   
}
void bin_dropoff(){           //Not started (Need camera stuff)

}

int main() {
    // Necessary initialization check
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return 1;
    }
    printf("Running main.\n");
    //------------------------ START OF CODE ------------------------



    //------------------------- END OF CODE -------------------------
    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
