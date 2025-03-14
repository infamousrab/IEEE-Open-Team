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
#define ACCEL_XOUT_REGH 0x3B // registers 3B-40 output gyro data in xyz directions 
#define ACCEL_YOUT_REGH 0x3D
#define ACCEL_ZOUT_REGH 0x3F
#define GYRO_XOUT_REGH 0x43 // registers 43-48 output gyro data in xyz directions 
#define GYRO_YOUT_REGH 0x45
#define GYRO_ZOUT_REGH 0x47
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C



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
char motor_state;

//IMU data points 
int i2c_handle;

// sensitivity factors for accel and gyro as floats for proper float division later
float accel_sens_factor = 16384; //units of LSB/g
float gyro_sens_factor = 131; //units of LSB/(º/s)

float accel_xout; 
float accel_yout; 
float accel_zout; 
float gyro_xout; 
float gyro_yout; 
float gyro_zout; 

float curr_robot_angle; 
float prev_robot_angle; 
float time_interval = 100; 

//*********************************************** GLOBAL VARIABLES ***********************************************
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
    float target_counts = (target_distance * CPR) / (PI * wheel_diameter);
    // move  motors until the target distance is reached
    while (abs(enc_r_count) < target_counts && abs(enc_l_count) < target_counts) {
        if (direction == 'F') {
            move_function(25, 'F');
        } else if (direction == 'B') {
            move_function(25, 'B'); 
        }
    }
    move_function(50, 'S');
}

void rotate_90_degrees(char direction) {
    // Reset encoder counts
    enc_r_count = 0;
    enc_l_count = 0;

    float target_distance = (PI * wheel_dist)/4;
    float target_counts = (target_distance * CPR) / (PI * wheel_diameter);

    // Right turn: Left wheel forward, right wheel backward
    if (direction == 'R') {
        while (abs(enc_l_count) < target_counts && abs(enc_r_count) < target_counts) {
            move_dc_motor('L', 50, 'F');
            move_dc_motor('R', 50, 'R');
            printf("Left: %d, Right: %d\n", enc_l_count, enc_r_count);
        }
    } 
    // Left turn: Right wheel forward, left wheel backward
    else if (direction == 'L') {
        while (abs(enc_l_count) < target_counts && abs(enc_r_count) < target_counts) {
            move_dc_motor('R', 50, 'F');
            move_dc_motor('L', 50, 'R');
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

// set full scale range of gyro output reads 
void set_gyro_fsr (uint8_t fs_sel) {
    uint8_t config = i2cReadByteData(i2c_handle, GYRO_CONFIG);
    // keep bits 7-5 and 2-0 by AND with 1 and clear bits 3 and 4 by AND with 0 then shift in fs_sel value by 3  
    config = (config & 0b11100111) | (fs_sel << 3); 
    i2cWriteByteData(i2c_handle, GYRO_CONFIG, config); 
}

// set full scale range of accel output reads 
void set_accel_fsr (uint8_t afs_sel) {
    uint8_t config = i2cReadByteData(i2c_handle, ACCEL_CONFIG); 
    // keep bits 7-5 and 2-0 by AND with 1 and clear bits 3 and 4 by AND with 0 then shift in fs_sel value by 3 
    config = (config & 0b11100111) | (afs_sel << 3); 
    i2cWriteByteData(i2c_handle, ACCEL_CONFIG, config); 
}

// reads high and low byte of the register, merges, and returns 
int16_t read_reg(int reg) {
    int16_t high = i2cReadByteData(i2c_handle, reg); // high byte of reg
    int16_t low = i2cReadByteData(i2c_handle, reg+1); //low byte of reg

    // merge high and low to single number 
    int16_t value = (high << 8) | low; 
    return value; 
}

void output_imu_data() {
    printf("Acceleration   (g) | x: %f y: %f z: %f\n", accel_xout, accel_yout, accel_zout); 
    printf("Angular rate (º/s) | x: %f y: %f z: %f\n", gyro_xout, gyro_yout, gyro_zout); 
}

void read_imu_data () {

    // hopefully converts raw gyro data to reasonable values 
    // increment registers by 2 to read each coordinate data value 
    gyro_xout = read_reg(GYRO_XOUT_REGH)/gyro_sens_factor;
    gyro_yout = read_reg(GYRO_YOUT_REGH)/gyro_sens_factor; 
    gyro_zout = read_reg(GYRO_ZOUT_REGH)/gyro_sens_factor; 

    //hopefully converts raw accelerator data to reasoanble values 
    accel_xout = read_reg(ACCEL_XOUT_REGH)/accel_sens_factor; 
    accel_yout = read_reg(ACCEL_YOUT_REGH)/accel_sens_factor; 
    accel_zout = read_reg(ACCEL_ZOUT_REGH)/accel_sens_factor; 

    //output_imu_data(); 

    // discrete integration on each callback 
    curr_robot_angle = prev_robot_angle + (time_interval/1000)*gyro_xout;
    prev_robot_angle = curr_robot_angle; 
    printf("gyro_x: %f\n", gyro_xout); 
    printf("gyro_y: %f\n", gyro_yout);
    printf("gyro_z: %f\n", gyro_zout); 
    printf("robot angle: %f\n", curr_robot_angle);

}

void robot_angle () {
    //assume an inital angle of 0 degrees  
    prev_robot_angle = 0;
    //take the angular rate over small time intervals (10ms) and integrate.

    // callback to read all imu data while in function 
    gpioSetTimerFunc(0, time_interval, read_imu_data); 
      
    sleep(5); 
    //rotate_90_degrees(direction); 
    //cancel timer at end of rotation 
    gpioSetTimerFunc(0, 500, NULL); 
}


//*************************************************** FUNCTIONS **************************************************

int main() {
    // Necessary initialization check
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return 1;
    }
    printf("Running main.\n");

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
    //gpioSetAlertFunc(ENCA_L, enc_l_callback); 
    //gpioSetAlertFunc(ENCA_R, enc_r_callback);
    //scl 3, sda 2

    //0 fsr for gyro gives sensitivity of ±250 (º/s) with sensitivity scale factor of 131 LSB/(º/s)
    set_gyro_fsr(0b00); 
    // 0 fsr for accel gives sensitivity of ±2g with sensivity scale factor of 16,384 LSB/g
    set_accel_fsr(0b00); 

    //rotate_robot('R'); 
    robot_angle(); 
 
    printf("Right wheel distance: %f\n", distance_travelled_r()); 
    printf("Left wheel distance: %f\n", distance_travelled_l());
    printf(" "); 

    //move_function(0, 'S'); 

    //------------------------- END OF CODE -------------------------
    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();
    return 0;
}