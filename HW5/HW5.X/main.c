/******************************************************************************
 *  PROGRAM COMMENTS
 *****************************************************************************/

/*
 * Aaron Weatherly
 * Northwestern University 
 * ME 433 Advanced Mechatronics
 * 
 * Homework 5:
 * Use the MCP23008 I/O expander to read from a pushbutton input and output to 
 * an LED. The LED is on by default and turns off when the button is pushed.
 * Communication between the PIC32MX250F128B (master) and the MCP23008 (slave)
 * is over 12C2.
 * 
 */


/******************************************************************************
 *  PREPROCESSOR COMMANDS
 *****************************************************************************/

#include<xc.h>                      // processor SFR definitions
#include<sys/attribs.h>             // __ISR macro
#include "i2c_master_noint.h"       // I2C functions

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

#define SLAVE_ADDR 0x20             // MCP23008 address
#define IODIR 0x00                  // address for input/output direction register
#define GPIO 0x09                   // address for the read register
#define OLAT 0x0A                   // address for the write register


/******************************************************************************
 *  DATA TYPE DEFINITIONS
 *****************************************************************************/



/******************************************************************************
 *  GLOBAL VARIABLES
 *****************************************************************************/



/******************************************************************************
 *  HELPER FUNCTION PROTOTYPES
 *****************************************************************************/

void initExpander(void);
void setExpander(unsigned char pin, unsigned char level);
unsigned char getExpander(void);


/******************************************************************************
 *  INTERRUPT SERVICE ROUTINES
 *****************************************************************************/



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

    // initialize I2C2 for the MCP23008
    initExpander();    
    unsigned char temp;
    
    __builtin_enable_interrupts();

    while(1) {
        setExpander(0,1);             // turn yellow LED on (set pin GP0 high) 
        temp = (getExpander()>>7);    // should see pin go low when button is pressed
        while(!temp) {                // while button is being pressed (reads low)
            setExpander(0,0);         // turn off yellow LED (set pin GP0 low)
            temp = (getExpander()>>7);
        }
    }
}


/******************************************************************************
 *  HELPER FUNCTIONS
 *****************************************************************************/

void initExpander(void) {
    ANSELBbits.ANSB2 = 0;                   // disable analog input on B2
    ANSELBbits.ANSB3 = 0;                   // disable analog input on B3
    i2c_master_setup();                     // set baud rate and turn on I2C2
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1 | 0);   // indicate write to slave
    i2c_master_send(IODIR);                 // writing to IODIR register
    i2c_master_send(0xF0);                  // GP0-GP3 as outputs, GP4-GP7 as inputs
    i2c_master_restart();                   // restart 
    i2c_master_send(SLAVE_ADDR << 1 | 0);   // indicate write to slave
    i2c_master_send(OLAT);                  // writing to OLAT register
    i2c_master_send(0x00);                  // turn all outputs off initially
    i2c_master_stop();                      // stop
}

void setExpander(unsigned char pin, unsigned char level) {
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1 | 0);   // indicate write to slave
    i2c_master_send(OLAT);                  // writing to OLAT register
    i2c_master_send((level << pin));        // turn pin on or off
    i2c_master_stop();                      // stop
}

unsigned char getExpander(void) {
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1 | 0);   // indicate write to slave
    i2c_master_send(GPIO);                  // reading from GPIO register
    i2c_master_restart();                   // restart sequence
    i2c_master_send(SLAVE_ADDR << 1 | 1);   // indicate read from slave
    unsigned char read = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(1);                      // master needs no more bytes
    i2c_master_stop();                      // stop 
    return read;
}