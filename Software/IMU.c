#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>

#define MPU6500_I2C_ADDR 0b1101000
#define GYRO_XOUT_REGH 0x43
#define GYRO_XOUT_REGL 0x44

int i2c_handle; //to handle i2c communication
float gyro_sens_factor = 131.0; //units of LSB/(º/s)
float gyro_offset = 0.0;    // will be found in calibration
float current_angle = 0.0;  // will be updated
struct timeval prev_time; //struct used to get accurate time back (both tv.sec and tv.usec)

// PID globals 
float pid_error = 0.0; 
float prev_pid_error = 0.0;

// Initializes i2c and pigpio (i2c wont work without pigpio)
void init_gyro() {
   /* if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        exit(1);
    } */
    i2c_handle = i2cOpen(1, MPU6500_I2C_ADDR, 0);
    if (i2c_handle < 0) {
        fprintf(stderr, "Failed to open I2C\n");
        gpioTerminate();
        exit(1);
    }
    // very neat function in sys/time to put time into timeval struct
    // sets up initialization of prev_time
    gettimeofday(&prev_time, NULL);
}
// Reads gyro raw data, converts to degrees/sec using senssitivity factor
float get_gyro_value() {
    uint8_t gyro_xout_high = i2cReadByteData(i2c_handle, GYRO_XOUT_REGH);
    uint8_t gyro_xout_low = i2cReadByteData(i2c_handle, GYRO_XOUT_REGL);
    int16_t raw_value = (int16_t)((gyro_xout_high << 8) | gyro_xout_low); //combine
    return (float)raw_value / gyro_sens_factor; //return converted gyro value as float
}
// Gyro calibration to find offset (averages gyro values for given number of samples when stationary)
void calibrate_gyro(int samples) {
    printf("Calibrating gyro... Keep sensor stationary.\n");
    float sum = 0.0;
    for (int i = 0; i < samples; i++) {
        sum += get_gyro_value();
        usleep(1000); // maybe find better sampling rate here??
    }
    gyro_offset = sum/samples;
    printf("Calibration complete. Gyro offset: %f\n", gyro_offset);
}
// Update current angle by integrating
void update_angle() {
    float gyro_value = get_gyro_value() - gyro_offset; //"correct" gyro value
    // initialize and get accurate current time to then use in delta t
    struct timeval current_time; 
    gettimeofday(&current_time, NULL);
    //integrate (find delta t, use to calculate current angle, over all time)
    double delta_t = (current_time.tv_sec - prev_time.tv_sec) + (current_time.tv_usec - prev_time.tv_usec)/1e6; //measure time difference precisely
    current_angle += gyro_value * delta_t; //calculate new angle using change in time and gyro value
    prev_time = current_time;
    printf("Gyro Rate: %f deg/s | Current Angle: %f degrees\n", gyro_value, current_angle);
}

// PID controller to provide PWM values to the motors for accurate rotation 
float rotate_PID_compute (double Kp, double Ki, double Kd, double set_angle, double dt) {
    pid_error = set_angle - current_angle; 
 
    float pid_integral;
    pid_integral += pid_error * dt; 
    // output = Kp * error + Ki * integral + Kd * derivative
    float output = Kp*pid_error + Ki*pid_integral + Kd*(pid_error - prev_pid_error)/dt;  

    // set previous error to current error
    prev_pid_error = pid_error;
    
    // set maxs and mins on output speeds
    if (output > 80.0) {
        output = 80.0;
    }
    if (output < 35.0) {
        output = 35.0; 
    }
    return output;
}

void cleanup() {
    i2cClose(i2c_handle);
    //gpioTerminate();
}
/*
int main() {
    init_gyro();      
    calibrate_gyro(1000); 
   
    while(1) {
        update_angle();   
        usleep(1000);   
    }
        
    cleanup();        
    return 0;
} 
*/
