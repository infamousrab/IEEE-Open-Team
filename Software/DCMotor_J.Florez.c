/***************************************************************************************************************************
 *                                         Raspberry Pi 4B GPIO Pinout Breakdown
 *                                           Assumes Pi USB is facing downward
 * 
 *                                     Physical Pin | GPIO Number | Function | Notes
 * -------------------------------------------------------------------------------------------------------------------------
 *                   Left Column (Odd Pins)                   |                Right Column (Even Pins)
 * -------------------------------------------------------------------------------------------------------------------------
 *  1   | -       | 3.3V Power           | Power Output       |  2   | -       | 5V Power            | Power Output
 *  3   | GPIO2   | I2C1 SDA             | I2C Data           |  4   | -       | 5V Power            | Power Output
 *  5   | GPIO3   | I2C1 SCL             | I2C Clock          |  6   | -       | GND                 | Ground
 *  7   | GPIO4   | GPIO General Purpose | Default: GPCLK0    |  8   | GPIO14  | UART0 TXD           | Transmit Pin
 *  9   | -       | GND                  | Ground             | 10   | GPIO15  | UART0 RXD           | Receive Pin
 * 11   | GPIO17  | GPIO General Purpose | Default: GPIO      | 12   | GPIO18  | PWM0/PCM_CLK        | PWM/Audio Clock
 * 13   | GPIO27  | GPIO General Purpose | Default: GPIO      | 14   | -       | GND                 | Ground
 * 15   | GPIO22  | GPIO General Purpose | Default: GPIO      | 16   | GPIO23  | GPIO General Purpose| Default: GPIO
 * 17   | -       | 3.3V Power           | Power Output       | 18   | GPIO24  | GPIO General Purpose| Default: GPIO
 * 19   | GPIO10  | SPI0 MOSI            | SPI Master Out     | 20   | -       | GND                 | Ground
 * 21   | GPIO9   | SPI0 MISO            | SPI Master In      | 22   | GPIO25  | GPIO General Purpose| Default: GPIO
 * 23   | GPIO11  | SPI0 SCLK            | SPI Clock          | 24   | GPIO8   | SPI0 CE0            | SPI Chip Enable 0
 * 25   | -       | GND                  | Ground             | 26   | GPIO7   | SPI0 CE1            | SPI Chip Enable 1
 * 27   | GPIO0   | ID EEPROM SDA        | ID EEPROM (I2C)    | 28   | GPIO1   | ID EEPROM SCL       | ID EEPROM (I2C)
 * 29   | GPIO5   | GPIO General Purpose | Default: GPIO      | 30   | -       | GND                 | Ground
 * 31   | GPIO6   | GPIO General Purpose | Default: GPIO      | 32   | GPIO12  | PWM0                | Default: GPIO
 * 33   | GPIO13  | PWM1                 | Default: GPIO      | 34   | -       | GND                 | Ground
 * 35   | GPIO19  | PWM1/PCM_FS          | PWM/Audio Sync     | 36   | GPIO16  | GPIO General Purpose| Default: GPIO
 * 37   | GPIO26  | GPIO General Purpose | Default: GPIO      | 38   | GPIO20  | PCM_DIN             | Audio Data In
 * 39   | -       | GND                  | Ground             | 40   | GPIO21  | PCM_DOUT            | Audio Data Out
 ***************************************************************************************************************************/

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

//*********************************************** GLOBAL VARIABLES ***********************************************
//************************************************** STRUCTURES **************************************************



//************************************************** STRUCTURES **************************************************
//*************************************************** FUNCTIONS **************************************************

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
void beacon_positioning(){ //Functioning (Hard-coded)
    move_function(24, 'B');
    usleep(450000);
    move_function(50, 'S');
    sleep(1);
    move_function(44, 'R');
    usleep(1000000);
    move_function(50, 'S');
    sleep(1);
    move_function(15, 'F');
    usleep(4000000);
    move_function(50, 'S');
    sleep(2);
    move_function(46, 'R');
    usleep(1100000);
    move_function(50, 'S');
    sleep(1);
    move_function(19, 'F');
    usleep(1200000);            //may need slight adjusting
    move_function(50, 'S');
    sleep(1);
}
void cube_pickups(){ //In-progress
    move_function(20, 'B');
    usleep(900000);
    move_function(50, 'S');
    sleep(1);
    move_function(54, 'L');
    usleep(11000000);
    move_function(50, 'S');
    sleep(1);
    move_function(20, 'B');
    usleep(3300000);
    move_function(50, 'S');
    sleep(1);

}
void lawnmower_path(){ //In progress
        //foward
        move_function(45, 1);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //rotate left
        move_function(30, -2);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //foward
        move_function(20, 1);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1);
        //rotate left
        move_function(30, -2);
        sleep(1);
        //brake
        move_function(50, 0);
        sleep(1); 
}
void lawnmower_path_cave(){   //Not Started
   
}
void bin_dropoff(){           //Not started (Need camera stuff)

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
    
    intake_functions(50, 'O');
    sleep(1);
    intake_functions(50, 'S');
    sleep(1);
    beacon_positioning();
    cube_pickups();

    //------------------------- END OF CODE -------------------------

    // Necessary library termination 
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
