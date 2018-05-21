#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c__master_noint.h"

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


void initExpander(){
    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}

void setExpander(char pin, char level) {
    i2c_master_start();
    i2c_master_send((0b0100000<<1)|0b00000000);
    i2c_master_send(0x0A) ;     //Output latch register, bit 0 is pin 0, a value of 1 is logic high
    i2c_master_send((level<<pin)|0b00000000); // writing the logic level to the pin
    i2c_master_stop();
}

char getExpander() {
    char c;
    i2c_master_start();
    i2c_master_send((0b0100000<<1)|0b00000000);
    i2c_master_send(0x09);     //address of the GPIO register
    i2c_master_restart();
    i2c_master_send((0b0100000<<1)|0b00000001);
    c = i2c_master_recv();
    i2c_master_ack(1); // make the ack so the slave knows we got it
    i2c_master_stop();
    return c;
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
    
    //TRIS and LAT commands
    TRISAbits.TRISA4 = 0b0;     //pin A4 is an output
    LATAbits.LATA4 = 0b1;       //A4 initially off
    TRISBbits.TRISB4 = 0b1;     //pin B4 is an input

    initExpander();
    __builtin_enable_interrupts();
    
    //configuring GP0-GP3 as outputs, and GP4-GP7 as inputs
    i2c_master_start();
    i2c_master_send((0b0100000<<1)|0b00000000);
    i2c_master_send(0x00);  //input/output direction register
    i2c_master_send(0b11110000);    //directions matching input and output as desired
    i2c_master_stop();
    
    //blinking the LED, first it is initialized to on
    setExpander(0,1);   //led is on GP0
    
    _CP0_SET_COUNT(0);
    double LedTime = 0;     //variable for the PIC's LED to blink
 while(1) {
            if( (getExpander()>>7) ==0 ){
                setExpander(0,1);
            }
            else {
            setExpander(0,0);
            }
            
            LedTime = LedTime + _CP0_GET_COUNT();
        if (LedTime > 5000000000) { //delay so the LED blinks at a perceptible rate
        LATAINV = 0b1 << 4;  //blinking the pic's LED to identify the code has not crashed 
        _CP0_SET_COUNT(0);
        LedTime=0;
        }
    }
}