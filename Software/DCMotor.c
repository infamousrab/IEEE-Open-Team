//*************************************************** INCLUDES ***************************************************
#include <stdio.h>        // Standard Input/Output functions
#include <stdlib.h>
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
#include <signal.h>
//*************************************************** INCLUDES ***************************************************
//*********************************************** GLOBAL VARIABLES ***********************************************
#define MPU6500_I2C_ADDR 0b1101000 //1101000 if AD0 is OFF
#define WHO_AM_I_REG 0x75

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
float PI = 3.14159265;
float wheel_dist = 5.25;  // Distance between wheels
//global enc count variables
volatile int enc_r_count = 0;
volatile int enc_l_count = 0;
int motor_state;
//*********************************************** GLOBAL VARIABLES ***********************************************
//*************************************************** FUNCTIONS **************************************************
// encoder callbacks that are passed the gpio, new level, and tick 

// Interrupt callback function for left encoder
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
void move_dc_motor(char PWM, int position, char output) {
    int PWM1, PWM2;
    if (position < 0 || position > 100) {
        printf("Position out of range\n"); 
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
    }
    // forward
    else if (output == 'F') {
        gpioWrite(PWM1, 0);
        gpioPWM(PWM2, position); 
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
        motor_state = 'f';
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
        motor_state = 'r';
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

void intake_functions(int position, char move) {
    if (move == 'F') {  
        move_dc_motor('F', position, 'R'); //Front intake

    } else if (move == 'N') {
        move_dc_motor('F', position, 'B'); //Front intake STOP
    }
     else if (move == 'O') { //BIN PICKUP OPEN
        move_dc_motor('C', position, 'F');  

    } else if (move == 'C') { //BIN PICKUP CLOSE
        move_dc_motor('C', position, 'R');

    } else if (move == 'S') { //BIN PICKUP STOP
        move_dc_motor('C', position, 'B');
    }
}

float distance_travelled_r() {
    return enc_r_count * PI * wheel_diameter / CPR; 
}

float distance_travelled_l() {
    return enc_l_count * PI * wheel_diameter / CPR;
}

// Move a given distance (FOWARD OR BACKWARD, in inches) [FOR SOME REASON OFF BY 2 INCHES]
void move_distance(float target_distance, char direction) {

    enc_r_count = 0;
    enc_l_count = 0;
    // calculate target counts 
    float offset_counts = (1.75 * CPR) / (PI * wheel_diameter); //fine-tune 1.75??
    float target_counts = (target_distance * CPR) / (PI * wheel_diameter) - offset_counts;
    // move  motors until the target distance is reached
    while (abs(enc_r_count) < target_counts && abs(enc_l_count) < target_counts) {
        if (direction == 'F') {
            move_function(20, 'F');
        } else if (direction == 'B') {
            move_function(20, 'B'); 
            
        }
    }
    move_function(0, 'S');
}

void rotate_90_degrees(char direction) {
    // Reset encoder counts
    enc_r_count = 0;
    enc_l_count = 0;

    float target_distance = (PI * wheel_dist)/4;
    float target_counts = (target_distance * CPR) / (PI * wheel_diameter);

    // Right turn: Left wheel forward, right wheel backward
    if (direction == 'R') {
        while (abs(enc_l_count) < target_counts) {
            move_dc_motor('L', 30, 'F');
            move_dc_motor('R', 30, 'R');
            printf("Left: %d, Right: %d\n", enc_l_count, enc_r_count);
        }
    } 
    // Left turn: Right wheel forward, left wheel backward
    else if (direction == 'L') {
        while (abs(enc_r_count) < target_counts) {
            move_dc_motor('R', 30, 'F');
            move_dc_motor('L', 30, 'R');
            printf("Left: %d, Right: %d\n", enc_l_count, enc_r_count);
        }
    }
    move_function(50, 'S'); // Stop the motors
}

void right_90deg(){
    sleep(1);
    move_function(45, 'R');
    sleep(1);
    move_function(45, 'S');
    sleep(1);

}
void left_90deg(){
    sleep(1);
    move_function(45, 'R');
    sleep(1);
    move_function(45, 'S');
    sleep(1);
}


//*************************************************** FUNCTIONS **************************************************

int main() {
    // Necessary initialization check
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return 1;
    }
    printf("Running main.\n");

    int i2c_handle;
    i2c_handle = i2cOpen(1, MPU6500_I2C_ADDR, 0);
    int who_am_i = i2cReadByteData(i2c_handle, WHO_AM_I_REG);
    printf("WHO_AM_I Register: 0x%X\n", who_am_i);
    
    //------------------------ START OF CODE ------------------------
    //Set GPIOs
    gpioSetMode(ENCA_L, PI_INPUT);
    gpioSetMode(ENCB_L, PI_INPUT); 
    gpioSetMode(ENCA_R, PI_INPUT); 
    gpioSetMode(ENCB_R, PI_INPUT); 

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
    move_distance(4, 'F');
    sleep(1);
    right_90deg();

    //move_distance(25, 'F');
    sleep(1);



    printf("Right wheel distance: %f\n", distance_travelled_r()); 
    printf("Left wheel distance: %f\n", distance_travelled_l());
    

    //------------------------- END OF CODE -------------------------
    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();
    return 0;
}