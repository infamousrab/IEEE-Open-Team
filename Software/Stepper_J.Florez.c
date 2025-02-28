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

#define STEP_PIN 20 
#define DIR_PIN  21  

//*********************************************** GLOBAL VARIABLES ***********************************************
//************************************************** STRUCTURES **************************************************



//************************************************** STRUCTURES **************************************************
//*************************************************** FUNCTIONS **************************************************

void stepMotor(int steps, int direction) {
    gpioWrite(DIR_PIN, direction); // direction of the stepper motor

    for (int i = 0; i < steps; i++) { //loop to continue moving steps in said direction
        gpioWrite(STEP_PIN, 1);
        time_sleep(0.001); // wait 100ms
        gpioWrite(STEP_PIN, 0);
        time_sleep(0.001);
    }
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

    gpioSetMode(STEP_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);

    //test (is 0 or 1 foward????)
 
    //stepMotor(200, 1); // Move 1 full turn (LEFT->DOWN)
    sleep(1);
    stepMotor(200, 0); // Move 1 full turn (RIGHT->UP)
    sleep(1);
    //------------------------- END OF CODE -------------------------

    // Necessary library termination
    printf("Terminating main.\n");
    gpioTerminate();

    return 0;
}
