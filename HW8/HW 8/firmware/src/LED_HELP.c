// functions to operate the ST7735 on the PIC32
// adapted from https://github.com/sumotoy/TFT_ST7735
// and https://github.com/adafruit/Adafruit-ST7735-Library

// pin connections:
// VCC - 3.3V
// GND - GND
// CS - B7
// RESET - 3.3V
// A0 - B15
// SDA - A1
// SCK - B14
// LED - 3.3V

// B8 is turned into SDI1 but is not used or connected to anything

#include <xc.h>
#include <math.h>
#include "LED_HELP.h"

void SPI1_init() {
  SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // SS is B7
  LATBbits.LATB7 = 1; // SS starts high

  // A0 / DAT pin
  ANSELBbits.ANSB15 = 0;
  TRISBbits.TRISB15 = 0;
  LATBbits.LATB15 = 0;

  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 0; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0; // clear the overflow bit
  SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1; // master operation
  SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(com);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat>>8);
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_init() {
    SPI1_init();
  int time = 0;
  LCD_command(ST7735_SWRESET);//software reset
  time = _CP0_GET_COUNT();
  while (_CP0_GET_COUNT() < time + 48000000/2/2) {}

	LCD_command(ST7735_SLPOUT);//exit sleep
  time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {}

	LCD_command(ST7735_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x01);
	LCD_data(0x2C);
	LCD_data(0x2D);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_FRMCTR2);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x01);
	LCD_data(0x2C);
	LCD_data(0x2D);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_FRMCTR3);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x01);
	LCD_data(0x2C);
	LCD_data(0x2D);
	LCD_data(0x01);
	LCD_data(0x2C);
	LCD_data(0x2D);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_INVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_PWCTR1);
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	LCD_data(0x84);//added auto mode
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_PWCTR2);
	LCD_data(0xC5);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command( ST7735_PWCTR3);
	LCD_data(0x0A);
	LCD_data(0x00);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command( ST7735_PWCTR4);
	LCD_data(0x8A);
	LCD_data(0x2A);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command( ST7735_PWCTR5);
	LCD_data(0x8A);
	LCD_data(0xEE);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_VMCTR1);
	LCD_data(0x0E);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_INVOFF);

	LCD_command(ST7735_MADCTL);
	LCD_data(0xC8);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_COLMOD);
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_CASET);
	LCD_data(0x00);
	LCD_data(0x00);
	LCD_data(0x00);
	LCD_data(0x7F);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_RASET);
	LCD_data(0x00);
	LCD_data(0x00);
	LCD_data(0x00);
	LCD_data(0x9F);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_GMCTRP1);
	LCD_data(0x02);
	LCD_data(0x1C);
	LCD_data(0x07);
	LCD_data(0x12);
	LCD_data(0x37);
	LCD_data(0x32);
	LCD_data(0x29);
	LCD_data(0x2D);
	LCD_data(0x29);
	LCD_data(0x25);
	LCD_data(0x2B);
	LCD_data(0x39);
	LCD_data(0x00);
	LCD_data(0x01);
	LCD_data(0x03);
	LCD_data(0x10);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_GMCTRN1);
	LCD_data(0x03);
	LCD_data(0x1D);
	LCD_data(0x07);
	LCD_data(0x06);
	LCD_data(0x2E);
	LCD_data(0x2C);
	LCD_data(0x29);
	LCD_data(0x2D);
	LCD_data(0x2E);
	LCD_data(0x2E);
	LCD_data(0x37);
	LCD_data(0x3F);
	LCD_data(0x00);
	LCD_data(0x00);
	LCD_data(0x02);
	LCD_data(0x10);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

	LCD_command(ST7735_NORON);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/100) {}

	LCD_command(ST7735_DISPON);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/10) {}

	LCD_command(ST7735_MADCTL); // rotation
    LCD_data(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
  // check boundary
  LCD_setAddr(x,y,x+1,y+1);
  LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
  LCD_command(ST7735_CASET); // Column
  LCD_data16(x0);
	LCD_data16(x1);

	LCD_command(ST7735_RASET); // Page
	LCD_data16(y0);
	LCD_data16(y1);

	LCD_command(ST7735_RAMWR); // Into RAM
}

void LCD_clearScreen(unsigned short color) {
  int i;
  LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
	for (i = 0;i < _GRAMSIZE; i++){
		LCD_data16(color);
	}
}

void  drawChar(short x ,short y ,char* mess ,short c1 ,short c2){      //draws characters from the ASCII table
	char row = *mess - 0x20 ;
	int col = 0;
    int indicate = 0;           //counter to allow characters to run onto new lines
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
            if ( x+col >128) {
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
    for (ll = 0; abs(ll) < abs(len); ll++) {
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

//new function for HW7, drawing a crosshair from an origin point
void drawCross (short x, short y, short len, short thick, signed short progx, signed short progy, short c1 , short c2 ) {
    int ll, tt;
    
    //neg x/y indicate if x or y values are negative. They are switched because of the way the IMU is mounted on my breadboard
    int negx = 1;
    int negy = 1;
    
    //checking if either progress is in negative axis
    if (progx<0){
        progx = -progx;
        negx = 0;
    }
    if (progy<0){
        progy = -progy;
        negy = 0;
    }

    //looping through each pixel in the cross hair and drawing one of two colors
    for (ll = 0; ll < len; ll++) {
        for (tt = 0; tt < thick; tt++){
            //clearing old pixels
            LCD_drawPixel(x+ll ,y+tt ,c1 );
            LCD_drawPixel(x-ll ,y+tt ,c1 );
            LCD_drawPixel(x+tt ,y+ll ,c1 );
            LCD_drawPixel(x+tt ,y-ll ,c1 );
            
            if (ll<progx){
                if (negx){
                    LCD_drawPixel(x-ll ,y+tt ,c2 );
                }else{
                    LCD_drawPixel(x+ll ,y+tt ,c2 );
                }
            }
            if (ll<progy){
                if (negy){
                    LCD_drawPixel(x+tt ,y-ll ,c2 );
                }else{
                    LCD_drawPixel(x+tt ,y+ll ,c2 );
                }
            }
        }
    }
}