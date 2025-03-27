//*************************************************** INCLUDES ***************************************************
#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>       // Raspberry Pi GPIO control library
#include <unistd.h>       // Provides sleep function
//*************************************************** INCLUDES ***************************************************

//*********************************************** GLOBAL VARIABLES ***********************************************
#define STEP 20
#define DIR 21
//*********************************************** GLOBAL VARIABLES ***********************************************

//*************************************************** FUNCTIONS **************************************************
void extend() {
    printf("Extending actuator...\n");
    gpioWrite(DIR, 1); 
    gpioWrite(STEP, 1);
    time_sleep(0.001);
    gpioWrite(STEP, 0); 

}

void retract() {
    printf("Retracting actuator...\n");
    gpioWrite(DIR, 0); 
    gpioWrite(STEP, 1);
    time_sleep(0.001);
    gpioWrite(STEP, 0); 
}



// int main() {
//     if (gpioInitialise() < 0) {
//         fprintf(stderr, "Failed to initialize pigpio. Make sure pigpiod is running.\n");
//         return 1;
//     }

//     printf("GPIO initialized. Running main program...\n");

//     // Set pin modes
//     gpioSetMode(STEP, PI_OUTPUT);
//     gpioSetMode(DIR, PI_OUTPUT); 

//     extend();
//     sleep(3); // Retract for 3 seconds

//     retract();
//     sleep(3); // Ensure actuator fully stops before exiting

//     // Cleanup
//     printf("Terminating program.\n");
//     gpioTerminate();
//     return 0;
// }
