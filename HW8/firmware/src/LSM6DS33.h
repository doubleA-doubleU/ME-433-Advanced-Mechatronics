// code for LSM6DS33 on the PIC32

#ifndef LSM6DS33_H__
#define LSM6DS33_H__

// LSM6DS33 registers
#define FUNC_CFG_ACCESS     	0x01 // embedded functions configuration
#define FIFO_CTRL_1             0x06 // FIFO configuration
#define FIFO_CTRL_2             0x07 // FIFO configuration
#define FIFO_CTRL_3             0x08 // FIFO configuration
#define FIFO_CTRL_4             0x09 // FIFO configuration
#define FIFO_CTRL_5             0x0A // FIFO configuration
#define ORIENT_CFG_G            0x0B // orientation configuration
#define INT1_CTRL               0x0D // INT1 pin control
#define INT2_CTRL               0x0E // INT2 pin control
#define WHO_AM_I                0x0F // accelerometer identity
#define CTRL1_XL                0x10 // accelerometer/gyroscope control
#define CTRL2_G                 0x11 // accelerometer/gyroscope control
#define CTRL3_C                 0x12 // accelerometer/gyroscope control
#define CTRL4_C                 0x13 // accelerometer/gyroscope control
#define CTRL5_C                 0x14 // accelerometer/gyroscope control
#define CTRL6_C                 0x15 // accelerometer/gyroscope control
#define CTRL7_G                 0x16 // accelerometer/gyroscope control
#define CTRL8_XL                0x17 // accelerometer/gyroscope control
#define CTRL9_XL                0x18 // accelerometer/gyroscope control
#define CTRL10_C                0x19 // accelerometer/gyroscope control
#define WAKE_UP_SRC             0x1B // interrupt register
#define TAP_SRC                 0x1C // interrupt register
#define D6D_SRC                 0x1D // interrupt register
#define STATUS_REG              0x1E // status data
#define OUT_TEMP_L              0x20 // temperature reading (least significant byte)
#define OUT_TEMP_H              0x21 // temperature reading (most significant byte)
#define OUTX_L_G                0x22 // gyroscope output
#define OUTX_H_G                0x23 // gyroscope output 
#define OUTY_L_G                0x24 // gyroscope output
#define OUTY_H_G                0x25 // gyroscope output
#define OUTZ_L_G                0x26 // gyroscope output
#define OUTZ_H_G                0x27 // gyroscope output
#define OUTX_L_XL               0x28 // accelerometer output
#define OUTX_H_XL               0x29 // accelerometer output
#define OUTY_L_XL               0x2A // accelerometer output
#define OUTY_H_XL               0x2B // accelerometer output
#define OUTZ_L_XL               0x2C // accelerometer output
#define OUTZ_H_XL               0x2D // accelerometer output
#define FIFO_STATUS1            0x3A // FIFO status
#define FIFO_STATUS2            0x3B // FIFO status
#define FIFO_STATUS3            0x3C // FIFO status
#define FIFO_STATUS4            0x3D // FIFO status
#define FIFO_DATA_OUT_L         0x3E // FIFO data output
#define FIFO_DATA_OUT_H         0x3F // FIFO data output
#define TIMESTAMP0_REG          0x40 // timestamp output
#define TIMESTAMP1_REG          0x41 // timestamp output
#define TIMESTAMP2_REG          0x42 // timestamp output
#define STEP_TIMESTAMP_L        0x49 // step counter timestamp
#define STEP_TIMESTAMP_H        0x4A // step counter timestamp
#define STEP_COUNTER_L          0x4B // step counter output
#define STEP_COUNTER_H          0x4C // step counter output
#define FUNC_SRC                0x53 // interrupt
#define TAP_CFG                 0x58 // interrupt
#define TAP_THS_6D              0x59 // interrupt
#define INT_DUR2                0x5A // interrupt
#define WAKE_UP_THS             0x5B // interrupt
#define WAKE_UP_DUR             0x5C // interrupt
#define FREE_FALL               0x5D // interrupt
#define MD1_CFG                 0x5E // interrupt
#define MD2_CFG                 0x5F // interrupt

#define ADDR                    0x6B // accelerometer chip address

void initAccel(void); // set up I2C2 and the accelerometer
void I2C_read_multiple(unsigned char reg, unsigned char * data, int length); // read from multiple registers in a row

#endif
