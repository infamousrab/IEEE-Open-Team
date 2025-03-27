//*************************************************** INCLUDES ***************************************************
#include <stdio.h>        // Standard Input/Output functions
#include <stdlib.h>
#include <pigpio.h>      // Raspberry Pi GPIO control library
#include <unistd.h>      // Provides access to the POSIX operating system API
#include <math.h>        // Math library for advanced mathematical functions
#include <signal.h>
#include "IMU.c"
#include <sys/time.h>
//*************************************************** INCLUDES ***************************************************
//*********************************************** GLOBAL VARIABLES ***********************************************
#define MPU6500_I2C_ADDR 0b1101000 //1101000 if AD0 is OFF
#define WHO_AM_I_REG 0x75
#define GYRO_XOUT_REGH 0x43
#define GYRO_XOUT_REGL 0x44

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
int ENCA_R = 23;
int ENCB_R = 24;
int ENCA_C = 25;
int ENCB_C = 8; 

// encoder/wheel variables 
float CPR = 600; // encoder counts per revolution 
float wheel_diameter = 3.14961;
float PI = 3.14159265;
float wheel_dist = 5.25;  // Distance between wheels
//global enc count variables
volatile int enc_r_count = 0;
volatile int enc_l_count = 0;
volatile int enc_c_count = 0;
char motor_state;
char motor_state2;
struct timeval pid_prev_time; 
//IMU stuff...

//Servo motor
int SERVO_PIN = 5;

//clamp motor 
float clamp_pitch = 0.0590551; 

//*********************************************** GLOBAL VARIABLES ***********************************************
//*************************************************** FUNCTIONS **************************************************
//Servo motor
void moveServo(int angle) {
    int pulseWidth = 500 + (angle * 2000) / 180; 
    gpioServo(SERVO_PIN, pulseWidth);
    sleep(1); 
} 

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

void enc_c_callback (int gpio, int level, uint32_t tick) {
    if (motor_state2 == 'O') {
        enc_c_count += 1;
    }
    else {
        enc_c_count -= 1;
    }
    printf("%d\n", enc_c_count); 
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
        motor_state2 = 'O'; 
        move_dc_motor('C', position, 'F');  

    } else if (move == 'C') { //BIN PICKUP CLOSE
        motor_state2 = 'C'; 
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
float distance_travelled_c() {
    return enc_c_count * CPR/clamp_pitch;
}

void intake_distance(float target_distance, char direction){
    enc_c_count = 0;
    float target_counts = (target_distance * CPR)/clamp_pitch;
    printf("Target Counts: %f", target_counts);
    while (abs(enc_c_count) < target_counts) {
        if (direction == 'O') {
            intake_functions(50, 'O');
            printf("Encoder count: %d \n", enc_c_count);
            

        } 
        else if (direction == 'C') {
            intake_functions(50, 'C');
            printf("Encoder count: %d \n", enc_c_count);
        }
    }
    intake_functions(0, 'S');
}


// Move a given distance (FOWARD OR BACKWARD, in inches) [Precise, but has some OFFSET]
void move_distance(float target_distance, char direction) {
    enc_r_count = 0;
    enc_l_count = 0;
    // calculate target counts 
    float offset_counts = (1.75 * CPR) / (PI * wheel_diameter); //fine-tune 1.75??
    float target_counts = (target_distance * CPR) / (PI * wheel_diameter) - offset_counts; 
    float motor_PWM; 
    float threshold_distance = 3.0;

    // move  motors until the target distance is reached
    while (abs(enc_r_count) < target_counts && abs(enc_l_count) < target_counts) {
        if (direction == 'F') {
            motor_PWM = 25; 
            move_function(motor_PWM, 'F'); //Fine-tune from 20-25 [May need to up with bins attached!]

            // if robot is within 1.5 inches of target distance, drop PWM
            if (target_distance - fabs(distance_travelled_l()) < threshold_distance && target_distance - fabs(distance_travelled_r()) < threshold_distance) {
                motor_PWM = 18;
                printf("Slowing down motor speed.");
                move_function(motor_PWM, 'F'); 
            }
            
        } else if (direction == 'B') {
            motor_PWM = 25; 
            move_function(motor_PWM, 'B');

            // if robot is within 1.5 inches of target distance, drop PWM
            if (target_distance - fabs(distance_travelled_l()) < threshold_distance && target_distance - fabs(distance_travelled_r()) < threshold_distance) {
                motor_PWM = 18;
                printf("Slowing down motor speed"); 
                move_function(motor_PWM, 'B'); 
            }
        }
    }
    move_function(0, 'S');
}

// rotate left at pid_PWM
void rotate_left(float pid_PWM) {
    move_dc_motor('R', pid_PWM, 'F');
    move_dc_motor('L', pid_PWM , 'R');
}

//rotate right at pid_PWM
void rotate_right(float pid_PWM) {
    move_dc_motor('L', pid_PWM, 'F'); 
    move_dc_motor('R', pid_PWM, 'R'); 
}

void rotate_90_degrees(char direction) {
    // Reset the gyro's current angle to 0
    current_angle = 0.0;
    gettimeofday(&prev_time, NULL);  //Reset time reference function
    init_gyro();
    // Calibrate gyro before starting rotation
    calibrate_gyro(1000); 

    double stable_time = 0.3; // robot angle should be stable for 300ms before breaking 
    double last_stable_time = 0.0; // tracks last time of stable error 

    //declare PID tuning constants; 
    double Kp;
    double Ki;
    double Kd; 
    // set target angle and PID tuning constants based on direction 
    float target_angle; 

        if (direction == 'L') {
            target_angle = 90.0;
            Kp = 7.0;
            Ki = 2;
            Kd = 0.001;
        }
        if (direction == 'R') {
            target_angle = -90.0; 
            Kp = 3.0;
            Ki = 0;
            Kd = 0.001;
        }
    
    // read and update robot current angle until within threshold for 0.3 s
    while (1) { 
        update_angle();  // Update the current_angle using the gyro

        // initialize and get accurate current time to then use in delta t
        struct timeval current_time; 
        gettimeofday(&current_time, NULL);

        //find delta t, use to input to PID controller
        double delta_t = (current_time.tv_sec - prev_time.tv_sec) + (current_time.tv_usec - prev_time.tv_usec)/1e6; //measure time difference precisely
        
        // Get PWM values from PID controller
        float pid_PWM = rotate_PID_compute(Kp, Ki, Kd, target_angle, delta_t);
        printf("pid_error: %f", pid_error); 
        printf("pid PWM: %f\n", pid_PWM);

        float scalar = 1.0; 
        float angle_error = fabs(target_angle - current_angle);
        float scalar_cap = 0.5;

        // when within 15 degrees, scale the PWM 
        if (angle_error < 15) {
            scalar = angle_error / 15; 
            
            // set scalar cap to avoid excessively low PWM values 
            if (scalar < scalar_cap) {
                scalar = scalar_cap; 
            }
        }

        float scaled_PWM = scalar * pid_PWM; 

        //overshoot/undershoot correction 
        if (direction == 'R') {

            // in case of overshoot, want to turn left 
            // less than to account for negative angle
            if (current_angle < target_angle) {
                rotate_left(scaled_PWM); 
            }
            // in case of undershoot, want to turn right
            if (current_angle > target_angle) {
                rotate_right(scaled_PWM); 
            }
        } 
        else if (direction == 'L') {
    
            // in case of overshoot, want to turn right 
            if (current_angle > target_angle) {
                rotate_right(scaled_PWM);
            }
            // in case of undershoot, want to turn left
            if (current_angle < target_angle) {
                rotate_left(scaled_PWM); 
            }
        }
        

        // if the angle is within the threshold, check if the angle has changed appreciably since the last check 
        if (fabs(target_angle - current_angle) < 2.5) {
            // If true, first time robot has been in the threshold region, so set last stable time to current time 
            if (last_stable_time == 0.0) {
                last_stable_time = current_time.tv_sec + current_time.tv_usec/1e6; 
            }
            // Check if robot has remained stable long enough. 
            else if ((current_time.tv_sec + current_time.tv_usec / 1e6) - last_stable_time > stable_time) {
                break; 
            }
        
            printf("last_stable_time %f | current_time %f\n | time difference %f\n", last_stable_time, current_time.tv_sec + current_time.tv_usec / 1e6, (current_time.tv_sec + current_time.tv_usec / 1e6) - last_stable_time); 
        }
        //Reset stability timer if robot moves outside the threshold 
        else if (fabs(target_angle - current_angle) > 2.5) {
            last_stable_time = 0.0;
        }
        
        usleep(1000); // small delay to avoid excessive loop speed
    }
    // stop the motors
    move_function(50, 'S'); 
    cleanup();
} 

//*************************************************** FUNCTIONS **************************************************



// int main() {
//     // Necessary initialization check
//     if (gpioInitialise() < 0) {
//         fprintf(stderr, "Failed to initialize pigpio\n");
//         return 1;
//     }
//     printf("Running main.\n");
    
//     /* 
//     int i2c_handle;
//     i2c_handle = i2cOpen(1, MPU6500_I2C_ADDR, 0);
//     int who_am_i = i2cReadByteData(i2c_handle, WHO_AM_I_REG);
//     printf("WHO_AM_I Register: 0x%X\n", who_am_i); //IMU TEST
//     */
    
//     //------------------------ START OF CODE ------------------------
//     //Set GPIOs
//     gpioSetMode(ENCA_L, PI_INPUT);
//     gpioSetMode(ENCB_L, PI_INPUT); 
//     gpioSetMode(ENCA_R, PI_INPUT); 
//     gpioSetMode(ENCB_R, PI_INPUT); 

//     gpioSetPWMfrequency(PWM1_L, 1000);
//     gpioSetPWMfrequency(PWM2_L, 1000);
//     gpioSetMode(PWM1_L, PI_OUTPUT);
//     gpioSetMode(PWM2_L, PI_OUTPUT);
//     gpioSetPWMfrequency(PWM1_R, 1000);
//     gpioSetPWMfrequency(PWM2_R, 1000);
//     gpioSetMode(PWM1_R, PI_OUTPUT);
//     gpioSetMode(PWM2_R, PI_OUTPUT);
//     gpioSetPWMfrequency(PWM1_C, 1000);
//     gpioSetPWMfrequency(PWM2_C, 1000);
//     gpioSetMode(PWM1_C, PI_OUTPUT);
//     gpioSetMode(PWM2_C, PI_OUTPUT);
//     gpioSetPWMfrequency(PWM1_F, 1000);
//     gpioSetPWMfrequency(PWM2_F, 1000);
//     gpioSetMode(PWM1_F, PI_OUTPUT);
//     gpioSetMode(PWM2_F, PI_OUTPUT);
    
//     //Encoder Callback functions
//     gpioSetAlertFunc(ENCA_L, enc_l_callback); 
//     gpioSetAlertFunc(ENCA_R, enc_r_callback);




//     //rotate_90_degrees('L'); 
//     //sleep(1);
    
//     //BEACON POSITIONING

//     move_distance(17.75, 'B'); 
//     sleep(1);
//     move_function(50, 'S'); 
//     sleep(1);
//     rotate_90_degrees('R'); 
//     sleep(1); 
//     move_distance(27, 'F'); 
//     sleep(1); 
//     moveServo(180);
//     moveServo(0);
    
//     printf("Right wheel distance: %f\n", distance_travelled_r()); 
//     printf("Left wheel distance: %f\n", distance_travelled_l());
    
//     //------------------------- END OF CODE -------------------------
//     // Necessary library termination
//     printf("Terminating main.\n");
//     gpioTerminate();
//     return 0;
// } 
