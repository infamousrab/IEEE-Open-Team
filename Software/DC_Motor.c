/***************************************************************************************************************************
 *                                         Raspberry Pi 4B GPIO Pinout Breakdown
 * 
 *                                 Physical Pin | GPIO Number | Function                  | Notes
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

#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>
 
int enableA = 18;
int PWM1 = 12;
int PWM2 = 13; 

// output:
// -2 | reverse/brake at speed PWM 
// -1 | brake low
//  0 | coast
//  1 | forward/brake at speed PWM

int move_drive_motor(int position, int output) {
    if (position < 0 || position > 100) {
        printf("Position out of range\n");
        return 1; 
    }
    position = (255*position / 100); 
    
    // reverse/brake at speed PWM
    if (output == -2) {
        //ENA = 1, ENB = 0, PWM1 = 0, PWM2 = PWM
        gpioWrite(enableA, 1); 
        //gpioWrite(enableB, 0);
        gpioWrite(PWM1, 0);
        gpioPWM(PWM2, position);
    }
    //brake low 
    else if (output == -1) {
        //ENA = 1, ENB = 0, PWM1 = 0, PWM2 = 0
        gpioWrite(enableA, 1); 
        //gpioWrite(enableB, 0);
        gpioWrite(PWM1, 0);
        gpioWrite(PWM2, 0);
    }
    //coast
    else if (output == 0) {
        //ENA = 0, ENB = 0, PWM1 = 0, PWM2 = 0
        gpioWrite(enableA, 0); 
        //gpioWrite(enableB, 0);
        gpioWrite(PWM1, 0);
        gpioWrite(PWM2, 0);
    }
    //forward/brale at speed PWM
    else if (output == 1) {
        //ENA = 1, ENB = 0, PWM1 = PWM, PWM2 = 0
        gpioWrite(enableA, 1); 
        //gpioWrite(enableB, 0);
        gpioPWM(PWM1, position);
        gpioWrite(PWM2, 0);
    }
}

int main() {
    if (gpioInitialise() < 0)
    {
        //initialization failed 
        printf(stderr, "Failed to initialize pigpio\n");
        return 1; 
    }
    gpioSetPWMfrequency(PWM1, 1000);
    gpioSetPWMfrequency(PWM2, 1000);
    gpioSetMode(enableA, PI_OUTPUT); 
    gpioSetMode(PWM1, PI_OUTPUT);
    gpioSetMode(PWM2, PI_OUTPUT);


    move_drive_motor(50, 1);
    sleep(1); 
    move_drive_motor(50, 0); 
    sleep(1);
    move_drive_motor(50, -1); 
    sleep(1);
    move_drive_motor(50, -2); 
    sleep(1);
    move_drive_motor(50, 0); //good to disable motor before exiting code

    gpioTerminate();

    return 0;
    
}
