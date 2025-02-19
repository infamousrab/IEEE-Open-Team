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

int move_dc_motor(int PWM, int position, int output) {
    int PWM1, PWM2;
    if (position < 0 || position > 100) {
        printf("Position out of range\n");
        return 1; 
    }
    position = (255*position / 100);

    if (PWM == 0){  //Set PWM to left motor
        PWM1 = PWM1_L;  
        PWM2 = PWM2_L;
    }
    else if (PWM == 1){ //Set PWM to right motor
        PWM1 = PWM1_R;
        PWM2 = PWM2_R;
    }
    else if (PWM == 2){ //Set PWM to front intake
        PWM1 = PWM1_F;
        PWM2 = PWM2_F;
    }
    else if (PWM == 3){ //Set PWM to cube pickup
        PWM1 = PWM1_C;
        PWM2 = PWM2_C;
    }

    //reverse
    if (output == -1) {
        gpioPWM(PWM1, position);
        gpioWrite(PWM2, 0);
    }

    // forward
    else if (output == 1) {
        gpioWrite(PWM1, 0);
        gpioPWM(PWM2, position);
    }

    //brake 
    else if (output == 0) {
        gpioWrite(PWM1, 0);
        gpioWrite(PWM2, 0);
    }


}

int move_function(int position, int move)
{
    //FULL FORWARD
    if (move == 1){
        move_dc_motor(0, position, 1);
        move_dc_motor(1, position, 1);
    }
    //FULL BRAKE
    else if (move == 0){
        move_dc_motor(0, position, 0);
        move_dc_motor(1, position, 0);
    }
    //FULL REVERSE
    else if (move == -1){
        move_dc_motor(0, position, -1);
        move_dc_motor(1, position, -1);
    }
    //ROTATE FUNCTIONS??? (To be done)
    //rotate right
    else if (move == 2){
        move_dc_motor(0, position, 1);
        move_dc_motor(1, position, -1);
    }
    //rotate left
    else if(move == -2){
        move_dc_motor(0, position, -1);
        move_dc_motor(1, position, 1);
    }
}
int intake_function(int position, int move) {
    if (move == 10) {
        move_dc_motor(2, position, -1);
    } else if (move == -10) {
        move_dc_motor(2, position, 0);
    }
     else if (move == 20) {
        move_dc_motor(3, position, 1);  //bin pickup?
    } else if (move == -20) {
        move_dc_motor(3, position, -1);
    } else if (move == -21) {
        move_dc_motor(3, position, 0);
    }
}
int lawnmower_path(){
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

    /*
    //front intake
    intake_function(50, 10);
    sleep(1);
    //front intake stop
    intake_function(50, -10);
    sleep(1);
    */

    lawnmower_path();
    sleep(1);


    //------------------------- END OF CODE -------------------------

    // Necessary library termination 
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
