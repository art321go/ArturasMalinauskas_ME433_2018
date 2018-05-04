
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h> 	//for sine wave plotting

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



#define CS LATBbits.LATB3       // chip select pin


// send a byte via spi and return the response
unsigned char SPI_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// initialize spi 
void SPI_init() {
  ANSELA = 0;
  ANSELB = 0;
  RPB2Rbits.RPB2R = 0b0011;
  SDI1Rbits.SDI1R = 0b0100;
  
  TRISBbits.TRISB3 = 0;
  CS = 1;
  
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 1;            // baud rate to 10 MHz [SPI1BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MODE32=0;
  SPI1CONbits.MODE16=0;
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.MSSEN = 0;
  SPI1CONbits.ON = 1;       // turn on spi
}

void setVoltage(char channel, int v) {
    CS=0;
	int t;
	t= channel << 15; //a is at the very end of the data transfer
	t = t | 0b0111000000000000;
	t = t | ((v& 0b1111111111) << 2);
	SPI_io(t >> 8);
    SPI_io(t & 0xFF);
    CS = 1;
}



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

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0b0;     //pin A4 is an output
    LATAbits.LATA4 = 0b1;
    
    TRISBbits.TRISB4 = 0b1;     //pin B4 is an input

    __builtin_enable_interrupts();
    
    SPI_init(); 
 
 int ii = 0;
 float f;
  
  while(1) {
    _CP0_SET_COUNT(0);
    
    //setting Sine Wave
	//f = 512 +512*sin(ii*2*3.1415*10/1000);  //should make a 10Hz sin wave)
    //setVoltage(0,f);
    //ii++;
    CS = 1;
	while(_CP0_GET_COUNT() < 24000) {;}
    _CP0_SET_COUNT(0);
    CS = 0;
    setVoltage(0,0);
    setVoltage(1,0);
    while(_CP0_GET_COUNT() < 24000) {;}
  }
}