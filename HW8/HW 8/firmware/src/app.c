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
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //math functions
#include<stdio.h>
#include "LED_HELP.h"      //given library for ST7735
#include "i2c_master_noint.h"    //library for I2C

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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
#define COUNTER 1200000     //counter for IMU  
#define LEN     14          //length of data array to be out put from IMU (2 temp, 6 gyro, 6 accel register values)
#define addy    0b1101011   //address for the IMU
#define REGIS    0x20        //address of OUT_TEMP_L register


//new i2c functions
void setReg(char reg, char val) {       //sets the value of an imu register 
    i2c_master_start();
    i2c_master_send( addy<<1 |0);
    i2c_master_send(reg) ;     //sending the location of the register
    i2c_master_send(val); // writing the value to the register
    i2c_master_stop();
}

void init_IMU(){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    setReg(0x10 ,0b10000010);       //CTRL1_XL 1000 is 1.66kHz     00 +-2g      10 100Hz filter
    setReg(0x11 ,0b10001000) ;      //CTRL2_G  1000 is 1.66kHz     10 is 1000 dps sensitivity         00 default
    setReg(0x12 ,0b00000100) ;      //CTRL3_C  default, with IF_INC enabled (2nd bit)
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length) {
    i2c_master_start(); // make the start bit
    i2c_master_send( (address<<1) | 0); //sending address of the IMU
    i2c_master_send(reg); //  read from desired register
    i2c_master_restart(); // make the restart bit
    i2c_master_send( (address<<1) | 1); //sending address of the IMU
    
    int ll = 0;
    for (ll = 0; ll < length-1; ll++) { 
        data[ll] = i2c_master_recv(); // saving the value returned
        i2c_master_ack(0); // make the ack so the slave knows we got it
    }
    data[length] = i2c_master_recv(); // saving the value returned
    i2c_master_ack(1); // make the ack so the slave knows we got it
    i2c_master_stop(); // make the stop bit        
}


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
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    //setting up IMU for I2C communication and setting up LCD controls
    init_IMU();
    LCD_init();
    //TRIS and LAT commands
    TRISAbits.TRISA4 = 0b0;     //pin A4 is an output
    LATAbits.LATA4 = 0b1;       //A4 initially off
    TRISBbits.TRISB4 = 0b1;     //user button is input
    __builtin_enable_interrupts(); 
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
            LCD_clearScreen(SECONDARY_COL);
            _CP0_SET_COUNT(0); //initializing the core timer to zero
            int USER = 1 ;      //button for user input, default value 1 is unpressed

            //checking WHO_AM_I register
            unsigned char r;
            i2c_master_start(); // make the start bit
            i2c_master_send( (addy<<1) | 0); //sending address of the IMU
            i2c_master_send(0x0F); //  read from desired WHO_AM_I_REGISTER
            i2c_master_restart(); // make the restart bit
            i2c_master_send( (addy<<1) | 1); // sending address
            r = i2c_master_recv(); // saving the value returned
            i2c_master_ack(1); // make the ack so the slave knows we got it
            i2c_master_stop(); // make the stop bit
            if (r != 105) {
                char oops [STRLONG];
                sprintf(oops, "ERROR   I am:%d !", r );
                drawString(28,10,oops, PRIMARY_COL , SECONDARY_COL);
                while (1) {;}
            }
            //

            //LCD crosshair setup
            char str [STRLONG];
            int count=0; 
            int height = 2;
            int length = 50;
            //drawProgressBar (14 , 80, length , height-1 , PRIMARY_COL , SECONDARY_COL); //X axis
            //drawProgressBar (64 , 30, height , length , PRIMARY_COL , SECONDARY_COL);   //Y axis
            //

            // setting up LED heartbeat for the PIC
             _CP0_SET_COUNT(0);
            double LedTime = 0;     //variable for the PIC's LED to blink
            //

    while(1) {
        
        //HW1 code working
//        while (_CP0_GET_COUNT() < 2400) {;} //delay by .5 ms for 1kHz total cycle
//	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
//	// remember the core timer runs at half the sysclk
//        USER = PORTBbits.RB4;
//        if (USER)
//        { LATAINV = 0b1 << 4; }
//        if (!USER)                  //if USER pressed, LED off
//        { LATAbits.LATA4 = 0b0;}
//        _CP0_SET_COUNT(0);
//        
        _CP0_SET_COUNT(0);
        
        //progress counter to check if LCD has crashed
        if (count > 99) {   //reseting the count at 100
            count =0;       //count will immediately be incremented to start value of 1
            //drawProgress(14, 144, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL ); //reseting progress bar
        } 
        count ++;
        //drawProgress(14, 144, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL );
        while (_CP0_GET_COUNT() < COUNTER){;}
        sprintf(str, "Screen check %d  ", count )   ;
        drawString(28,0,str, PRIMARY_COL , SECONDARY_COL);  
        //
        
        //reseting values for the gyro
        unsigned char imu_raw [LEN];
        signed short imu_  [LEN/2];
        signed int imu_p  [LEN/2];    //adjusted IMU data to a percentage of 65535, the max value of the short
        char checkx [STRLONG];
        char checky [STRLONG];
        //
        
        //reading IMU data
        I2C_read_multiple(addy, REGIS, imu_raw, LEN);
        int mm =0;
        int nn =0;
        while (mm < LEN/2 ){        //populating an array of signed shorts with the high and low data values
            imu_[mm] = ( (imu_raw[nn+1] << 8) |  (imu_raw[nn])  );//creating a short of the values by shifting the high register bits and "or"ing the low bits
            imu_p[mm] = ( ( ( signed int ) (imu_[mm] ) ) * 100) / 32768 ; 
            mm ++;
            nn += 2;
        }        
        
        sprintf(checkx, "X: %d    %d   ", imu_[4], imu_p[4] )   ;
        sprintf(checky, "Y: %d    %d   ", imu_[5], imu_p[5] )   ;
        drawString(28,10,checkx, PRIMARY_COL , SECONDARY_COL); 
        drawString(28,20,checky, PRIMARY_COL , SECONDARY_COL); 
        //
        
        //drawing the x and y acceleration on the crosshair
        drawCross(64, 80, length, height, imu_p[4], imu_p[5], PRIMARY_COL, THIRD_COL);
        
        //drawing like an etch a sketch
        if (PORTBbits.RB4){
            LCD_drawPixel(64-imu_p[4] ,80-imu_p[5] ,BLUE );
            LCD_drawPixel(64-imu_p[4] ,81-imu_p[5] ,BLUE );
            LCD_drawPixel(65-imu_p[4] ,81-imu_p[5] ,BLUE );
            LCD_drawPixel(65-imu_p[4] ,80-imu_p[5] ,BLUE );
        }
    
        //delay so the LED blinks at a perceptible rate that isnt annoying
        LedTime = LedTime + _CP0_GET_COUNT();
        if (LedTime > 5000000) { 
            LATAINV = 0b1 << 4;  //blinking the pic's LED to identify the code has not crashed 
            LedTime=0;
        }
        //
    
    
    
    
    }
        
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
