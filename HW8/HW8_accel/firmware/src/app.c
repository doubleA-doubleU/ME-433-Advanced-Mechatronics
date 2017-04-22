/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "ILI9163C.h"               // LCD functions
#include "i2c_master_noint.h"       // I2C functions
#include "LSM6DS33.h"               // accelerometer functions
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
unsigned char msg[20]; // char array for LCD messages
unsigned char array[14]; // char array for accelerometer data
short temperature=0, gyroX=0, gyroY=0, gyroZ=0, accelX=0, accelY=0, accelZ=0; // readings

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    // initialize SPI1, I2C2, Accelerometer, and LCD screen
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    initAccel();
    
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
    display_string(msg,4,4,WHITE);          // print to LCD
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // erase previous x and y acceleration bars
            if (accelX > 0) {
                draw_bar(64 - accelX / 500, 64, BLACK, accelX / 500, 2);
            } else {
                draw_bar(64, 64, BLACK, -accelX / 500, 2);
            }
            if (accelY > 0) {
                draw_bar(64, 64 - accelY / 500, BLACK, 2, accelY / 500);
            } else {
                draw_bar(64, 64, BLACK, 2, -accelY / 500);
            }
            draw_bar(64, 64, WHITE, 2, 2); // center point

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
                draw_bar(64 - accelX / 500, 64, WHITE, accelX / 500, 2);
            } else {
                draw_bar(64, 64, WHITE, -accelX / 500, 2);
            }
            if (accelY > 0) {
                draw_bar(64, 64 - accelY / 500, WHITE, 2, accelY / 500);
            } else {
                draw_bar(64, 64, WHITE, 2, -accelY / 500);
            }
            _CP0_SET_COUNT(0); // set timer to zero
            while (_CP0_GET_COUNT() < 2400000) {;} // wait 100 ms (10Hz)
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
