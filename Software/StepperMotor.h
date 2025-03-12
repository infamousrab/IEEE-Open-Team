#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

//*************************************************** INCLUDES ***************************************************
#include <stdio.h>       // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
//*************************************************** INCLUDES ***************************************************

//*********************************************** GLOBAL VARIABLES ***********************************************
#define STEP_PIN 20 
#define DIR_PIN  21  
//*********************************************** GLOBAL VARIABLES ***********************************************

//*************************************************** FUNCTIONS **************************************************
void stepMotor(int steps, int direction);
//*************************************************** FUNCTIONS **************************************************

#endif // STEPPERMOTOR_H
