//*************************************************** INCLUDES ***************************************************
#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
//*************************************************** INCLUDES ***************************************************
//*********************************************** GLOBAL VARIABLES ***********************************************
#define STEP_PIN 20 
#define DIR_PIN  21  
//*********************************************** GLOBAL VARIABLES ***********************************************
//*************************************************** FUNCTIONS **************************************************
void stepMotor(int steps, int direction) {
    gpioWrite(DIR_PIN, direction); // direction of the stepper motor

    for (int i = 0; i < steps; i++) { //loop to continue moving steps in said direction
        gpioWrite(STEP_PIN, 1);
        time_sleep(0.001); // wait 100ms
        gpioWrite(STEP_PIN, 0);
        time_sleep(0.001);
    }
}
//*************************************************** FUNCTIONS **************************************************
int main() {
    // Necessary initialization check
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return 1;
    }
    printf("Running main.\n");
    //------------------------ START OF CODE ------------------------
    gpioSetMode(STEP_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);
    //stepMotor(600, 1); // Move 1 full turn (LEFT->DOWN)
    sleep(1);
    stepMotor(600, 0); // Move 1 full turn (RIGHT->UP)
    sleep(1);
    //------------------------- END OF CODE -------------------------
    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
