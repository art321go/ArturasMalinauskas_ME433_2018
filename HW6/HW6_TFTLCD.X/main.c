#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //math functions
#include<stdio.h>
#include "ST7735.h"      //given library for ST7735

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

void  drawChar(short x ,short y ,char* mess ,short c1 ,short c2){      //draws characters from the ASCII table
	char row = *mess - 0x20 ;
	int col = 0;
	for (col = 0; col < 5; col++) {
		char pixels = ASCII[row][col];
		int j = 0;
		for (j=0; j<8; j++) {
            if ( x+col <128 && y+j < 160 ) {      //checking the pixel is drawn in the range of the LCD screen
                if( (pixels >>j) & 1 == 1 ) {                
				LCD_drawPixel(x+col ,y+j ,c1 );         //draw pixel is defined in St7735.c
                }else{
				LCD_drawPixel(x+col ,y+j ,c2 );
                }
            }
        }
    }
}                           //Need to write a message, must have a null character, \0 to end strings



void drawString(short x ,short y ,char* message ,short c1 ,short c2 ) {           //draws the string of characters
	int i = 0;
	while (message[i] != 0){ 
        drawChar(x+5*i ,y ,&message[i] ,c1 ,c2 );    //x+5*i to write new characters at correct spacing. Different font size requires greater spacing between characters and different ascii table (except integer doubling where multiple pixels can be drawn for each one specified in the original font)
	i++ ; 
	}
}

void drawProgressBar (short x ,short y , short len , short h ,short c1 , short c2 ) {
    int ll, hh;
    for (ll = 0; ll < len; ll++) {
        LCD_drawPixel(x+ll ,y , c1 );
        LCD_drawPixel(x+ll ,y+1 , c1 ); //double thickness bar
        LCD_drawPixel(x+ll ,y+h-1 , c1 );
        LCD_drawPixel(x+ll ,y+h , c1 );
        for (hh = 0; hh < h; hh++){
            if ( (hh < 2) | (hh  > h-2) | ll<2 | ll>len-3 ){
                LCD_drawPixel(x+ll ,y+hh ,c1 );
            }else{
                LCD_drawPixel(x+ll ,y+hh , c2 );
            }
        }
    }
}

void drawProgress (short x, short y, short len, short h, short prog, short c1 , short c2 ) {
    int ll, hh;
    for (ll = 0; ll < len; ll++) {
        for (hh = 0; hh < h; hh++){
            if (ll < prog) {
                LCD_drawPixel(x+ll ,y+hh ,c1 );
            } else{
                LCD_drawPixel(x+ll ,y+hh ,c2 );
            }
        }
    }
}

#define STRLONG 80       //length of string
#define GREEN2    0xA670
#define PRIMARY_COL GREEN2  //color definitions
#define SECONDARY_COL WHITE
#define COUNTER 2400000     //frequency of the counter (10Hz)

int main () {
    LCD_init();
    LCD_clearScreen(SECONDARY_COL);
    char str [STRLONG];
    char fps [STRLONG];
    int count=1; 
    int height = 16;
    int length = 108;
    float frames= 0;  //initial value
    drawProgressBar (10 , 64, length , height , PRIMARY_COL , SECONDARY_COL);
    while (1){  
        _CP0_SET_COUNT(0);
        if (count > 99) {   //reseting the count at 100
            count =0;       //count will immediately be incremented to start value of 1
            sprintf(str, "Hi! Countup: %d   ", 1);
            
            frames=0;
            drawString(28,32,str, PRIMARY_COL , SECONDARY_COL);
            drawString(14,100,fps, PRIMARY_COL , SECONDARY_COL);
            drawProgress(14, 68, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL ); //reseting progress bar
        } 
        count ++;
        frames = _CP0_GET_COUNT();
        sprintf(str, "Hi! Countup: %d", count);        //will add countdown %d
        drawString(28,32,str, PRIMARY_COL , SECONDARY_COL );
        drawProgress(14, 68, length-8, height-8, count, PRIMARY_COL , SECONDARY_COL );
        frames = _CP0_GET_COUNT() - frames;         //counting time between bar drawing
        
        sprintf(fps, "FPS = %5.2f", frames*100/COUNTER);
        drawString(14,100,fps, PRIMARY_COL , SECONDARY_COL);
        while (_CP0_GET_COUNT() < COUNTER){;}
    }
    


}

