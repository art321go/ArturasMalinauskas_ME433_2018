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
			if( (pixels >>j) & 1 == 1 ) {
                if ( x+col <128 && y+j < 160 )      //checking the pixel is drawn in the range of the LCD screen
                {
				LCD_drawPixel(x+col ,y+j ,c1 );         //draw pixel is defined in St7735.c
                }
			else
				LCD_drawPixel(x+col ,y+j ,c2 );
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


//void drawProgressBar (x ,y ,h ,len1 ,color1 ,len2 ,c2 )
//Make pixel lines
#define STRLONG 80       //length of string

int main () {
    LCD_init();
    LCD_clearScreen(WHITE);
    char str [STRLONG];
    sprintf(str, "Hello World");        //will add countdown %d
    drawString(28,32,str, MAGENTA, YELLOW);


}

