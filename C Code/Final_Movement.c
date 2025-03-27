#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
#include <stdlib.h>      // Standard library
#include <sys/time.h>   // Good library for accurate time (calculating delta t)
#include <stdint.h>     // Standard int library
#include "dcTest1.c"   //DC Motor Movement functions
#include "Actuator.c" //Stepper Motor Movement function

//include LED and April Tag detect C header files

//Once LED is detected ON, begin all portions of Course

void beacon_positioning(){ //PRETTY MUCH DONE (~90% success)
    move_distance(17.5, 'B'); 
    sleep(1);
    rotate_90_degrees('R'); 
    sleep(1); 
    move_distance(27.5, 'F'); 
    sleep(1); 
    moveServo(180);
    moveServo(0);
}
void cube_pickups(){ //In-progress (Need DC Motor Encoder)
    move_distance(28, 'B');
    rotate_90_degrees('R');
    move_distance(15, 'B');
    sleep(1);
    rotate_90_degrees('L');
    extend();
    sleep(2);
    move_distance(13, 'B');
    sleep(1);
    retract();
    sleep(3);
    move_distance(7, 'F');


}
void lawnmower_path(){ //In progress (PROTOTYPE)
    //move_distance(29.5, 'F');
    //usleep(500000);
    //rotate_90_degrees('L');
    // usleep(500000);
    //move_distance(12, 'F');
    // usleep(500000);
    //rotate_90_degrees('L');
    // usleep(500000);
    // move_distance(26, 'F');
    // usleep(500000);
    // rotate_90_degrees('R');
    // usleep(500000);
    // move_distance(13, 'F');
    // usleep(500000);
    // rotate_90_degrees('R');
    // usleep(500000);
    // move_distance(11, 'F');
    // usleep(500000);
    // rotate_90_degrees('L');
    // usleep(500000);
    // move_distance(23, 'F');
    // usleep(500000);

}
void lawnmower_path_cave(){  // In progress (PROTOTYPE)
   
    //usleep(500000);
    //rotate_90_degrees('L');
    //usleep(500000);
    //move_distance(12, 'F');
    //usleep(500000);
    //rotate_90_degrees('R');
    //usleep(500000);
   // move_distance(8, 'F');
    //usleep(500000);
    // rotate_90_degrees('R');
    //usleep(500000);
    // move_distance(30, 'F');
    //usleep(500000);
    // rotate_90_degrees('L');
    // usleep(500000);
    // move_distance(9, 'F');
    // usleep(500000);
    // rotate_90_degrees('L');
    // usleep(500000);
    // move_distance(28, 'F');
    // usleep(500000);
    // rotate_90_degrees('L');
    // usleep(500000);
    // move_distance(15, 'F');
    // usleep(500000);
    // rotate_90_degrees('L'); //STALLED, NOOOO!!!
    //usleep(500000);
    // move_distance(13, 'F');
    // usleep(500000);
    // rotate_90_degrees('R'); //OVERSHOOTS BAD, NOOO!!!
    // usleep(500000);

    // move_distance(52 , 'F');

    // sleep(1);

}
void bin_dropoff(){      //"Need Camera" (or Move foward?? lift bins if possible)   

}

int main() {
    // Necessary initialization check
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return 1;
    }
    printf("Running main.\n");
    //------------------------ START OF CODE ------------------------
    //     gpioSetMode(ENCA_L, PI_INPUT);
    gpioSetMode(ENCA_L, PI_INPUT);
    gpioSetMode(ENCB_L, PI_INPUT); 
    gpioSetMode(ENCA_R, PI_INPUT); 
    gpioSetMode(ENCB_R, PI_INPUT); 

    gpioSetMode(ENCA_C, PI_INPUT);
    gpioSetMode(ENCB_C, PI_INPUT); 
    
 

    gpioSetPWMfrequency(PWM1_L, 1000);
    gpioSetPWMfrequency(PWM2_L, 1000);
    gpioSetMode(PWM1_L, PI_OUTPUT);
    gpioSetMode(PWM2_L, PI_OUTPUT);
    gpioSetPWMfrequency(PWM1_R, 1000);
    gpioSetPWMfrequency(PWM2_R, 1000);
    gpioSetMode(PWM1_R, PI_OUTPUT);
    gpioSetMode(PWM2_R, PI_OUTPUT);
    gpioSetPWMfrequency(PWM1_C, 1000);
    gpioSetPWMfrequency(PWM2_C, 1000);
    gpioSetMode(PWM1_C, PI_OUTPUT);
    gpioSetMode(PWM2_C, PI_OUTPUT);
    gpioSetPWMfrequency(PWM1_F, 1000);
    gpioSetPWMfrequency(PWM2_F, 1000);
    gpioSetMode(PWM1_F, PI_OUTPUT);
    gpioSetMode(PWM2_F, PI_OUTPUT);
    
    //Encoder Callback functions
    gpioSetAlertFunc(ENCA_L, enc_l_callback); 
    gpioSetAlertFunc(ENCA_R, enc_r_callback);
    gpioSetAlertFunc(ENCA_F, enc_c_callback); 


    // beacon_positioning();
    // sleep(1);
    // cube_pickups();
    // sleep(1);
    

    // lawnmower_path();
    
    lawnmower_path_cave();

    // intake_distance(2, 'O');
    // sleep(1); 
    // intake_functions(0, 'N'); 
    //sleep(1);
    //intake_functions(0, 'S');



    //------------------------- END OF CODE -------------------------
    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
