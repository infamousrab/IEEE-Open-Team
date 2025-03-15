#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
#include <stdlib.h>      // Standard library
#include <sys/time.h>   // Good library for accurate time (calculating delta t)
#include <stdint.h>     // Standard int library
#include "DCMotor.h"   //DC Motor Movement functions
#include "StepperMotor.h" //Stepper Motor Movement function

//include LED and April Tag detect C header files

//Once LED is detected ON, begin all portions of Course

void beacon_positioning(){ //In-progress (Hard-coded)

}
void cube_pickups(){ //In-progress
  

}
void lawnmower_path(){ //In progress

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
