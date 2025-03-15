#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>

#define MPU6500_I2C_ADDR 0b1101000
#define GYRO_XOUT_REGH 0x43
#define GYRO_XOUT_REGL 0x44

// Global variables
extern int i2c_handle; // To handle I2C communication
extern float gyro_sens_factor; // Units of LSB/(º/s)
extern float gyro_offset;    // Will be found in calibration
extern float current_angle;  // Will be updated
extern struct timeval prev_time; // Struct used to get accurate time back (both tv.sec and tv.usec)
extern double pid_error; 
extern double prev_pid_error; 
// Function declarations
void init_gyro(); // Initializes I2C and pigpio
float get_gyro_value(); // Reads gyro raw data and converts to degrees/sec
void calibrate_gyro(int samples); // Gyro calibration to find offset
void update_angle(); // Update current angle by integrating
void cleanup(); // Cleanup resources
#endif // GYRO_H
