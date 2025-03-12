#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>

// Function prototypes
void enc_l_callback(int gpio, int level, uint32_t tick);
void enc_r_callback(int gpio, int level, uint32_t tick);
void move_dc_motor(char PWM, int position, char output);
void move_function(int position, char move);
void intake_functions(int position, char move);
float distance_travelled_r();
float distance_travelled_l();

#endif // DCMOTOR_H