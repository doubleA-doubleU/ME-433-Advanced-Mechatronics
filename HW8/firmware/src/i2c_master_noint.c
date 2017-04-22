// I2C Master utilities, 400 kHz, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// Change I2C2 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

#include<xc.h>              // processor SFR definitions
#include "i2c_master_noint.h" 

void i2c_master_setup(void) {
    I2C2BRG = 0x35;         // I2CxBRG = [1/(2*Fsck) - PGD]*Pbclk - 2 = 53 = 0x35
                            // Fsck = 400kHz, PGD = 104 ns, Pbclk = 48 MHz
    I2C2CONbits.ON = 1;     // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;                // send the start bit
    while(I2C2CONbits.SEN) { ; }        // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;               // send a restart 
    while(I2C2CONbits.RSEN) { ; }       // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
    I2C2TRN = byte;                     // if an address, bit 0 = 0 for write, 1 for read
    while(I2C2STATbits.TRSTAT) { ; }    // wait for the transmission to finish
    if(I2C2STATbits.ACKSTAT) {          // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
    }
}

unsigned char i2c_master_recv(void) {   // receive a byte from the slave
    I2C2CONbits.RCEN = 1;               // start receiving data
    while(!I2C2STATbits.RBF) { ; }      // wait to receive the data
    return I2C2RCV;                     // read and return the data
}

void i2c_master_ack(int val) {          // sends ACK = 0 (slave should send another byte)
                                        // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;            // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;              // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }      // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {            // send a STOP:
    I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
    while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void initExpander(void) {                   // initialize MCP23008 pin expander
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

void setExpander(unsigned char pin, unsigned char level) { // set outputs pin on MCP23008
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1 | 0);   // indicate write to slave
    i2c_master_send(OLAT);                  // writing to OLAT register
    i2c_master_send((level << pin));        // turn pin on or off
    i2c_master_stop();                      // stop
}

unsigned char getExpander(void) {           // read input pin on MCP23008
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