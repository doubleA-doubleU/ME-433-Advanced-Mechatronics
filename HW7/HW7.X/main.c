/******************************************************************************
 *  PROGRAM COMMENTS
 *****************************************************************************/

/*
 * Aaron Weatherly
 * Northwestern University 
 * ME 433 Advanced Mechatronics
 * 
 * Homework 7:
 * Read accelerometer and gyroscope data from LSM6DS33 and print to LCD
 */


/******************************************************************************
 *  PREPROCESSOR COMMANDS
 *****************************************************************************/

#include <xc.h>                     // processor SFR definitions
#include <sys/attribs.h>            // __ISR macro
#include <stdio.h>                  // sprintf, etc.
#include "ILI9163C.h"               // LCD functions
#include "i2c_master_noint.h"       // I2C functions
#include "LSM6DS33.h"               // accelerometer functions

// DEVCFG0
#pragma config DEBUG = OFF          // no debugging
#pragma config JTAGEN = OFF         // no jtag
#pragma config ICESEL = ICS_PGx1    // use PGED1 and PGEC1
#pragma config PWP = OFF            // no write protect
#pragma config BWP = OFF            // no boot write protect
#pragma config CP = OFF             // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL       // use primary oscillator with pll
#pragma config FSOSCEN = OFF        // turn off secondary oscillator
#pragma config IESO = OFF           // no switching clocks
#pragma config POSCMOD = HS         // high speed crystal mode
#pragma config OSCIOFNC = OFF       // free up secondary osc pins
#pragma config FPBDIV = DIV_1       // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD       // do not enable clock switch
#pragma config WDTPS = PS1048576    // slowest wdt
#pragma config WINDIS = OFF         // no wdt window
#pragma config FWDTEN = OFF         // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2     // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24     // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2     // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2     // divide 8MHz input clock by 2, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON          // USB clock on

// DEVCFG3
#pragma config USERID = 0           // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF       // allow multiple reconfigurations
#pragma config IOL1WAY = OFF        // allow multiple reconfigurations
#pragma config FUSBIDIO = ON        // USB pins controlled by USB module
#pragma config FVBUSONIO = ON       // USB BUSON controlled by USB module


/******************************************************************************
 *  DATA TYPE DEFINITIONS
 *****************************************************************************/

// N/A

/******************************************************************************
 *  GLOBAL VARIABLES
 *****************************************************************************/

unsigned char msg[20]; // char array for LCD messages
unsigned char array[14]; // char array for accelerometer data
short temperature=0, gyroX=0, gyroY=0, gyroZ=0, accelX=0, accelY=0, accelZ=0; // readings


/******************************************************************************
 *  HELPER FUNCTION PROTOTYPES
 *****************************************************************************/

// N/A

/******************************************************************************
 *  INTERRUPT SERVICE ROUTINES
 *****************************************************************************/

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Timer2(void) {
    // erase previous x and y acceleration bars
    if (accelX > 0) {
        draw_bar(64-accelX/500,64,WHITE,accelX/500,2);
    } else {
        draw_bar(64,64,WHITE,-accelX/500,2);
    }    
    if (accelY > 0) {
        draw_bar(64,64-accelY/500,WHITE,2,accelY/500);
    } else {
        draw_bar(64,64,WHITE,2,-accelY/500);
    }
    draw_bar(64,64,BLACK,2,2); // center point
    
    // read current accelerometer data
    I2C_read_multiple(OUT_TEMP_L, array, 14);
    
    // update variables
    temperature = array[1] << 8 | array[0];
    gyroX = array[3] << 8 | array[2];
    gyroY = array[5] << 8 | array[4];
    gyroZ = array[7] << 8 | array[6];
    accelX = array[9] << 8 | array[8];
    accelY = array[11] << 8 | array[10];
    accelZ = array[13] << 8 | array[12];
    
    // draw current x and y acceleration bars
    if (accelX > 0) {
        draw_bar(64-accelX/500,64,BLACK,accelX/500,2);
    } else {
        draw_bar(64,64,BLACK,-accelX/500,2);
    }    
    if (accelY > 0) {
        draw_bar(64,64-accelY/500,BLACK,2,accelY/500);
    } else {
        draw_bar(64,64,BLACK,2,-accelY/500);
    }
       
    IFS0bits.T2IF = 0; // clear interrupt flag
}


/******************************************************************************
 *  MAIN FUNCTION
 *****************************************************************************/

int main() {
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // initialize SPI1, I2C2, Accelerometer, and LCD screen
    SPI1_init();
    LCD_init();
    LCD_clearScreen(WHITE);
    initAccel();
    
    // initialize ISR - TMR2 at 10 Hz
    T2CONbits.TCKPS = 0b111;// Timer2 prescaler N=256 (1:256)
	PR2 = 18749;			// period = (PR2+1) * N * 20.8333 ns = 0.1 s, 10 Hz
	TMR2 = 0;				// initial TMR2 count is 0
	T2CONbits.ON = 1;		// turn on Timer2
    IPC2bits.T2IP = 5;      // priority
	IPC2bits.T2IS = 0;      // subpriority
	IFS0bits.T2IF = 0;      // clear interrupt flag
	IEC0bits.T2IE = 1;      // enable interrupt
    
    // confirm connection to accelerometer, print to LCD
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(ADDR << 1 | 0);         // indicate write to slave
    i2c_master_send(WHO_AM_I);              // reading from WHO_AM_I register
    i2c_master_restart();                   // restart sequence
    i2c_master_send(ADDR << 1 | 1);         // indicate read from slave
    unsigned char who = i2c_master_recv();  // receive a byte from the bus
    i2c_master_ack(1);                      // master needs no more bytes
    i2c_master_stop();                      // stop 
    sprintf(msg,"WHO_AM_I = %d",who);       // print to string
    display_string(msg,4,4,BLACK);          // print to LCD
        
    __builtin_enable_interrupts();

    while (1) { ; } // infinite loop, waiting on timer 2 interrupt
}


/******************************************************************************
 *  HELPER FUNCTIONS
 *****************************************************************************/

// N/A