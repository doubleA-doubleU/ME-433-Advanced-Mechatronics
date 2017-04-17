/******************************************************************************
 *  PROGRAM COMMENTS
 *****************************************************************************/

/*
 * Aaron Weatherly
 * Northwestern University 
 * ME 433 Advanced Mechatronics
 * 
 * Homework 6:
 * The PIC32MX250F128B writes to the ILI9163C LCD screen via SPI1.
 */


/******************************************************************************
 *  PREPROCESSOR COMMANDS
 *****************************************************************************/

#include <xc.h>                     // processor SFR definitions
#include <sys/attribs.h>            // __ISR macro
#include <stdio.h>                  // sprintf...
#include "ILI9163C.h"               // LCD functions

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

#define BAR_WIDTH 25


/******************************************************************************
 *  DATA TYPE DEFINITIONS
 *****************************************************************************/



/******************************************************************************
 *  GLOBAL VARIABLES
 *****************************************************************************/



/******************************************************************************
 *  HELPER FUNCTION PROTOTYPES
 *****************************************************************************/

void display_character(unsigned char c, short x, short y, int color);
void display_string(unsigned char * msg, short x, short y, int color);
void draw_bar(short x, short y, int color, short len, short w);


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

    // initialize SPI1 and LCD screen
    SPI1_init();
    LCD_init();
    LCD_clearScreen(WHITE); // green background
    draw_bar(13,31,BLACK,102,1); // progress bar top border
    draw_bar(13,31,BLACK,1,BAR_WIDTH+2); // progress bar left border
    draw_bar(115,31,BLACK,1,BAR_WIDTH+2); // progress bar right border
    draw_bar(13,32+BAR_WIDTH,BLACK,102,1); // progress bar bottom border
    
    // declare variables
    char msg[20];
    int n = 0;
    int t;
    float fps;
    _CP0_SET_COUNT(0); // set time to zero
    
    __builtin_enable_interrupts();

    while(1) {     
        // write message in black characters (green background by default)
        sprintf(msg, "Hello World! %3d",n);
        display_string(msg,16,16,BLUE);
        
        // draw progress bar in black
        draw_bar(14+n,32,GREEN,1,BAR_WIDTH);
        
        // calculate and display frames per second
        t=_CP0_GET_COUNT(); // in core timer ticks
        fps = (((float) n)*24000000.0)/t;
        sprintf(msg,"%4.2f FPS",fps);
        display_string(msg,16,104,RED);
        
        // delay to meet 5Hz goal
        while (_CP0_GET_COUNT() < t + 4300000) {;}
        
        // increase counter up to 100, then reset to zero
        n = n + 1;
        if (n == 101) {
            n = 0;
            draw_bar(14,32,WHITE,101,BAR_WIDTH); // "erase" the progress bar
            _CP0_SET_COUNT(0); // reset time
        }
    }
}


/******************************************************************************
 *  HELPER FUNCTIONS
 *****************************************************************************/

void display_character(unsigned char c, short x, short y, int color) {
    char row = c - 0x20;
    int i, j;
    for (i=0; i<=4; i++) {
        if ((x+i) < 128) {
            for (j=0; j<=7; j++) {
                if ((y+j) < 128) {
                    LCD_drawPixel(x+i, y+j, WHITE);
                    if ((ASCII[row][i] >> j) & 1) {
                        LCD_drawPixel(x+i, y+j, color);
                    }
                }
            }
        }
    }
}

void display_string(unsigned char * msg, short x, short y, int color) {
    int k = 0;
    while (msg[k] != 0) {
        display_character(msg[k], x+(6*k), y, color);
        k = k + 1;
    }
}

void draw_bar(short x, short y, int color, short len, short w) {
    int l,m;
    for (l=0; l<len; l++) {
        for (m=0; m<w; m++) {
            LCD_drawPixel(x+l, y+m, color);
        } 
    }
}