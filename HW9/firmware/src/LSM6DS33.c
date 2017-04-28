// functions to operate the LSM6DS33 on the PIC32

// pin connections:
// VDD - NC
// VIN - 3.3V
// GND - GND
// SDA - B2(SDA2)
// SCL - B3(SCL2)
// SDO - NC
// CS - 3.3V
// INT2 - NC
// INT1 - NC

#include <xc.h>
#include "LSM6DS33.h"
#include "i2c_master_noint.h"

void initAccel(void) {
    ANSELBbits.ANSB2 = 0;           // disable analog input on B2
    ANSELBbits.ANSB3 = 0;           // disable analog input on B3
    i2c_master_setup();             // set baud rate and turn on I2C2
    i2c_master_start();             // begin the start sequence
    i2c_master_send(ADDR << 1 | 0); // indicate write to slave
    i2c_master_send(CTRL1_XL);      // writing to CTRL1_XL register
    i2c_master_send(0b10000010);    // accelerometer sample rate of 1.66 kHz, with 2g sensitivity, and 100 Hz filter
    i2c_master_restart();           // restart 
    i2c_master_send(ADDR << 1 | 0); // indicate write to slave
    i2c_master_send(CTRL2_G);       // writing to CTRL2_G register
    i2c_master_send(0b10001000);    // gyroscope sample rate of 1.66 kHz, with 1000 dps sensitivity
    i2c_master_restart();           // restart 
    i2c_master_send(ADDR << 1 | 0); // indicate write to slave
    i2c_master_send(CTRL3_C);       // writing to CTRL3_C register
    i2c_master_send(0b00000100);    // enable sequential read
    i2c_master_stop();              // stop
}

void I2C_read_multiple(unsigned char reg, unsigned char * array, int length) {
    i2c_master_start();                     // begin the start sequence
    i2c_master_send(ADDR << 1 | 0);         // indicate write to slave
    i2c_master_send(reg);                   // first register to read from
    i2c_master_restart();                   // restart sequence
    i2c_master_send(ADDR << 1 | 1);         // indicate read from slave
    int i;
    for (i=0; i<length; i++) {
        array[i] = i2c_master_recv();        // receive a byte from the bus
        if (i < (length - 1)) {
            i2c_master_ack(0);                  // master needs another byte
        }        
    }    
    i2c_master_ack(1);                      // master needs no more bytes    
    i2c_master_stop();                      // stop 
}