/******************************************************************************
 *  PROGRAM COMMENTS
 *****************************************************************************/

/*
 * Aaron Weatherly
 * Northwestern University 
 * ME 433 Advanced Mechatronics
 * 
 * Homework 4:
 * The PIC32MX250F128B is the master, utilizing SPI1. The MCP4902 DAC outputs a 
 * 10Hz sine wave on VoutA and a 5Hz triangle wave on VoutB, updating the values
 * 1000 times a second. The ISR is 2 kHz so that each time through only one of 
 * the voltages is calculated and sent to the DAC. They are both updated at 1000
 * times a second, but are 0.5 ms out of phase with one another.
 */


/******************************************************************************
 *  PREPROCESSOR COMMANDS
 *****************************************************************************/

#include<xc.h>                      // processor SFR definitions
#include<sys/attribs.h>             // __ISR macro
#include<math.h>                    // for sine function

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

#define CS LATBbits.LATB15          // B15 is chip select pin


/******************************************************************************
 *  DATA TYPE DEFINITIONS
 *****************************************************************************/



/******************************************************************************
 *  GLOBAL VARIABLES
 *****************************************************************************/

static volatile unsigned char channel = 0;
static volatile unsigned int i = 0;
static volatile float t = 0;


/******************************************************************************
 *  HELPER FUNCTION PROTOTYPES
 *****************************************************************************/

void initTMR2(void);
void initSPI1(void);


/******************************************************************************
 *  INTERRUPT SERVICE ROUTINES
 *****************************************************************************/

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Timer2(void) {
    float temp;
    unsigned char voltage, write1, write2, garbage;    
    t = ((float) i)/2000.0;                     // time in seconds
    if (channel == 0) {                         // voltage for channel A between 0 and 255 as 10 Hz sine wave
        temp = 128.0*sin(20.0*3.14159*t) + 128.0; // temporary variable to calculate voltage as a float
        voltage = temp;                         // store voltage as char
        write1 = 0b01110000 | (voltage >> 4);   // config bits plus the first 4 voltage bits
        write2 = (voltage << 4) | 0b00000000;   // second 4 voltage bits plus 4 zero bits
        channel = 1;                            // swap channels each time through ISR
    }
    else {                                      // voltage for channel B between 0 and 255 as 5 Hz sawtooth wave (t will reset at 100 ms)
        temp = 2550*t;                          // temporary variable to calculate voltage as a float
        voltage = temp;                         // store voltage as char
        write1 = 0b11110000 | (voltage >> 4);   // config bits plus the first 4 voltage bits
        write2 = (voltage << 4) | 0b00000000;   // second 4 voltage bits plus 4 zero bits
        channel = 0;                            // swap channels each time through ISR
    }
    CS = 0;                                     // command being sent
    SPI1BUF = write1;                           // send 1st half of voltage command to DAC
    while (!SPI1STATbits.SPIRBF){ ; }           // wait for the "response"
    garbage = SPI1BUF;                          // read garbage in buffer 
    SPI1BUF = write2;                           // send 2nd half of voltage command to DAC
    while (!SPI1STATbits.SPIRBF){ ; }           // wait for the "response"
    garbage = SPI1BUF;                          // read garbage in buffer 
    CS = 1;                                     // command is complete
    i++;                                        // increase counter (in increments of 0.5 ms)
    if (i > 200) {                              
        i = 0;                                  // reset counter every 100 ms (10 Hz)
        channel = 1;                            // set channel to 1 so that sine wave is continuous
    }
    IFS0bits.T2IF = 0;                          // clear interrupt flag
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

    // initialize SPI1
    initSPI1();
    
    // initialize timer 2 interrupt
    initTMR2();
        
    __builtin_enable_interrupts();

    while(1) { ; } // infinite loop, waiting on timer 2 interrupt
}


/******************************************************************************
 *  HELPER FUNCTIONS
 *****************************************************************************/

void initTMR2(void) {
    T2CONbits.TCKPS = 0;    // Timer2 prescaler N=1 (1:1)
	PR2 = 23999;			// period = (PR2+1) * N * 20.8333 ns = 0.5 ms, 2 kHz
	TMR2 = 0;				// initial TMR2 count is 0
	T2CONbits.ON = 1;		// turn on Timer2
    IPC2bits.T2IP = 5;      // priority
	IPC2bits.T2IS = 0;      // subpriority
	IFS0bits.T2IF = 0;      // clear interrupt flag
	IEC0bits.T2IE = 1;      // enable interrupt
}

void initSPI1(void) {           // set up spi 1 (SCK1 is B14 by default)
    TRISBbits.TRISB15 = 0;      // B15 is output used for CS
    CS = 1;                     // set CS high (no command being sent)
    RPB8Rbits.RPB8R = 0b0011;   // B8 is SDO1
    SDI1Rbits.SDI1R = 0b0000;   // SDI1 is A1 (not used though, DAC is one way)
    SPI1CON = 0;                // turn off the spi module and reset it
    SPI1BUF;                    // clear the rx buffer by reading from it
    SPI1BRG = 0x1;              // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1] (0x1 for 12 MHz, 0x95F for 10 kHz))
    SPI1STATbits.SPIROV = 0;    // clear the overflow bit
    SPI1CONbits.CKE = 1;        // data changes when clock goes from high to low (since CKP is 0)
    SPI1CONbits.MSTEN = 1;      // master operation
    SPI1CONbits.ON = 1;         // turn on spi 1    
}