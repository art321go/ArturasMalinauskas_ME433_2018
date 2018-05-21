#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //math functions
#include<stdio.h>
#include "LEDHELP.h"      //given library for ST7735
#include"i2c__master_noint.h"    //library for I2C

// DEVCFG0
#pragma config DEBUG = 0b0 // no debugging
#pragma config JTAGEN = 0b0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b111111111 // no write protect
#pragma config BWP = 0b1 // no boot write protect
#pragma config CP = 0b1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0b0 // turn off secondary oscillator
#pragma config IESO = 0b0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 0b1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b10 // do not enable clock switch
#pragma config WDTPS = 0b10100 // use slowest wdt
#pragma config WINDIS = 0b1 // wdt no window mode
#pragma config FWDTEN = 0b0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0b0 // USB clock on

// DEVCFG3
#pragma config USERID = 0b0110011001100110 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0b0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0b0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0b1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0b1 // USB BUSON controlled by USB module

#define COUNTER 2400000     //counter for progress bar  
#define addy    0b1101011   //address for the IMU

int main () {
    
    //I2C setup
    
    __builtin_disable_interrupts();
    init_i2c();
    __builtin_enable_interrupts();
    
 
    //LCD debugging setup
    LCD_init();
    LCD_clearScreen(SECONDARY_COL);
    char str [STRLONG];
    char fps [STRLONG];
    int count=1; 
    int height = 16;
    int length = 108;
    float frames= 0;  //initial value
    drawProgressBar (10 , 64, length , height , PRIMARY_COL , SECONDARY_COL);
    //end of LCD debugging setup
    
    
    while (1){  
        
        
        //progress bar to check if code crashed
        _CP0_SET_COUNT(0);
        if (count > 99) {   //reseting the count at 100
            count =0;       //count will immediately be incremented to start value of 1
            frames=0;
            drawString(14,100,fps, PRIMARY_COL , SECONDARY_COL);
            drawProgress(14, 68, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL ); //reseting progress bar
        } 
        count ++;
        frames = _CP0_GET_COUNT();
        drawProgress(14, 68, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL );
        frames = _CP0_GET_COUNT() - frames;         //counting time between bar drawing        
        sprintf(fps, "FPS = %5.2f", frames*100/COUNTER);
        drawString(14,100,fps, PRIMARY_COL , SECONDARY_COL);
        while (_CP0_GET_COUNT() < COUNTER){;}
    }
    
}
