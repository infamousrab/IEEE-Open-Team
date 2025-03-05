
//*************************************************** INCLUDES ***************************************************

#include <stdio.h>        // Standard Input/Output functions
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions

//*************************************************** INCLUDES ***************************************************
//*********************************************** GLOBAL VARIABLES ***********************************************

//left dc motor
int PWM1_L = 13;
int PWM2_L = 19;
//right dc motor (flipped PWM due to PCB)
int PWM1_R = 18;
int PWM2_R = 12;
//cube pickup motor
int PWM1_C = 9;
int PWM2_C = 10;
//front intake motor
int PWM1_F = 26;
int PWM2_F = 11;

// encoder gpios 
int ENCA_L = 14;
int ENCB_L = 15;
int ENCA_R = 24;
int ENCB_R = 25;

// encoder/wheel variables 
float CPR = 600; // encoder counts per revolution 
float wheel_diameter = 3.14961;

volatile int enc_r_count = 0;
volatile int enc_l_count = 0;

char motor_state; 

//*********************************************** GLOBAL VARIABLES ***********************************************
//************************************************** STRUCTURES **************************************************



//************************************************** STRUCTURES **************************************************
//*************************************************** FUNCTIONS **************************************************

// encoder callbacks that are passed the gpio, new level, and tick 
void enc_l_callback (int gpio, int level, uint32_t tick) {
    if (motor_state == 'f') {
        enc_l_count += 1;
    }
    else {
        enc_l_count -= 1;
    }
    printf("%d\n", enc_l_count); 
}

void enc_r_callback (int gpio, int level, uint32_t tick) {
    if (motor_state == 'f') {
        enc_r_count += 1;
    }
    else {
        enc_r_count -= 1;
    }
    printf("%d\n", enc_r_count); 
}

int move_dc_motor(char PWM, int position, char output) {
    int PWM1, PWM2;
    if (position < 0 || position > 100) {
        printf("Position out of range\n");
        return 1; 
    }
    position = (255*position / 100);

    if (PWM == 'L'){  //Set PWM to left motor
        PWM1 = PWM1_L;  
        PWM2 = PWM2_L;
    }
    else if (PWM == 'R'){ //Set PWM to right motor
        PWM1 = PWM1_R;
        PWM2 = PWM2_R;
    }
    else if (PWM == 'F'){ //Set PWM to front intake
        PWM1 = PWM1_F;
        PWM2 = PWM2_F;
    }
    else if (PWM == 'C'){ //Set PWM to cube pickup
        PWM1 = PWM1_C;
        PWM2 = PWM2_C;
    }
    //reverse
    if (output == 'R') {
        gpioPWM(PWM1, position);
        gpioWrite(PWM2, 0);
        motor_state = 'r'; 
    }
    // forward
    else if (output == 'F') {
        gpioWrite(PWM1, 0);
        gpioPWM(PWM2, position);
        motor_state = 'f'; 
    }
    //brake 
    else if (output == 'B') {
        gpioWrite(PWM1, 0);
        gpioWrite(PWM2, 0);
    }
}

void move_function(int position, char move)
{
    //FULL FORWARD
    if (move == 'F'){
        move_dc_motor('L', position, 'F');
        move_dc_motor('R', position, 'F');
    }
    //FULL BRAKE
    else if (move == 'S'){
        move_dc_motor('L', position, 'B');
        move_dc_motor('R', position, 'B');
    }
    //FULL REVERSE
    else if (move == 'B'){
        move_dc_motor('L', position, 'R');
        move_dc_motor('R', position, 'R');
    }
    //ROTATE FUNCTIONS 
    //rotate right
    else if (move == 'R'){
        move_dc_motor('L', position, 'F');
        move_dc_motor('R', position, 'R');
    }
    //rotate left
    else if(move == 'L'){
        move_dc_motor('L', position, 'R');
        move_dc_motor('R', position, 'F');
    }
}

float distance_travelled_r() {
    return enc_r_count * 3.14159 * wheel_diameter / CPR; 
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
    gpioSetMode(ENCA_L, PI_INPUT);
    gpioSetMode(ENCB_L, PI_INPUT); 
    gpioSetMode(ENCA_R, PI_INPUT); 
    gpioSetMode(ENCB_R, PI_INPUT); 
    
    motor_state = 'f'; 

    gpioSetAlertFunc(ENCA_R, enc_r_callback);
    gpioSetAlertFunc(ENCB_R, enc_r_callback);
    
    sleep(10);
    printf("%f", distance_travelled_r()); 
    
    //------------------------- END OF CODE -------------------------

    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}