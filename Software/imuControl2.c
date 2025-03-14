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
#define GYRO_XOUT_REGL 0x44
#define GYRO_YOUT_REGH 0x45
#define GYRO_YOUT_REGL 0x46
#define GYRO_ZOUT_REGH 0x47
#define GYRO_ZOUT_REGL 0x48
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

//IMU data points 
int i2c_handle;

// sensitivity factors for accel and gyro as floats for proper float division later
float accel_sens_factor = 16384.0; //units of LSB/g
float gyro_sens_factor = 131.0; //units of LSB/(º/s)

float accel_xout; 
float accel_yout; 
float accel_zout; 
float gyro_xout; 
float gyro_yout; 
float gyro_zout; 

float curr_robot_angle; 
float prev_robot_angle; 
float time_interval = 10; 

//gyro calibration 
float calc_gyro_x_offset; 
float offset_sum = 0; 
int sample_count = 0; 
float gyro_x_offset = -3.515344; 

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

//merges high and low bytes of registers
float get_imu_val(uint8_t high_byte, uint8_t low_byte, float sens_factor) {

    int16_t raw_value = (int16_t)((high_byte << 8) | low_byte);

    //printf("High byte: %d, Low byte:%d\n", high_byte, low_byte); 
    //printf("Raw gyro value %d\n\n", raw_value);  
    return (float)raw_value/sens_factor; 
    
}

void output_imu_data() {
    printf("Acceleration   (g) | x: %f y: %f z: %f\n", accel_xout, accel_yout, accel_zout); 
    printf("Angular rate (º/s) | x: %f y: %f z: %f\n", gyro_xout, gyro_yout, gyro_zout); 
}

void read_imu_data () {

    // hopefully converts raw gyro data to reasonable values 
    uint8_t gyro_xout_h = i2cReadByteData(i2c_handle, GYRO_XOUT_REGH); 
    uint8_t gyro_xout_l = i2cReadByteData(i2c_handle, GYRO_XOUT_REGL); 
    
    float gyro_xout = get_imu_val(gyro_xout_h, gyro_xout_l, gyro_sens_factor); 
    printf("Gyro_xout: %f\n", gyro_xout); 

    uint8_t gyro_yout_h = i2cReadByteData(i2c_handle, GYRO_YOUT_REGH); 
    uint8_t gyro_yout_l = i2cReadByteData(i2c_handle, GYRO_YOUT_REGL); 

    float gyro_yout = get_imu_val(gyro_yout_h, gyro_yout_l, gyro_sens_factor); 
    printf("Gyro_yout: %f\n", gyro_yout); 

    uint8_t gyro_zout_h = i2cReadByteData(i2c_handle, GYRO_ZOUT_REGH); 
    uint8_t gyro_zout_l = i2cReadByteData(i2c_handle, GYRO_ZOUT_REGL); 

    float gyro_zout = get_imu_val(gyro_zout_h, gyro_zout_l, gyro_sens_factor); 
    printf("Gyro_zout: %f\n", gyro_zout); 
    
    /*
    //hopefully converts raw accelerator data to reasoanble values 
    accel_xout = read_reg(ACCEL_XOUT_REGH)/accel_sens_factor; 
    accel_yout = read_reg(ACCEL_YOUT_REGH)/accel_sens_factor; 
    accel_zout = read_reg(ACCEL_ZOUT_REGH)/accel_sens_factor; 
    */
    //output_imu_data(); 

    printf("previous robot angle: %f\n", prev_robot_angle); 
    // discrete integration on each callback 
    curr_robot_angle = prev_robot_angle + (time_interval/1000.0)*(gyro_xout - gyro_x_offset);
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
      
    sleep(10); 
    //rotate_90_degrees(direction); 
    //cancel timer at end of rotation 
    gpioSetTimerFunc(0, time_interval, read_imu_data); 
}

// reading x gyro values and summing to calibrate the gyro 
void read_gyro_x() {
    uint8_t gyro_xout_h = i2cReadByteData(i2c_handle, GYRO_XOUT_REGH); 
    uint8_t gyro_xout_l = i2cReadByteData(i2c_handle, GYRO_XOUT_REGL); 
    
    float gyro_xout = get_imu_val(gyro_xout_h, gyro_xout_l, gyro_sens_factor); 

    offset_sum += gyro_xout;
    sample_count += 1; 

}

// determine gyro offset by taking samples for 2 minutes 
void calibrate_gyro() {
    // determine gyro offset

    gpioSetTimerFunc(0, 10, read_gyro_x); 

    sleep(120); 

    calc_gyro_x_offset = (float)offset_sum / sample_count;    
}