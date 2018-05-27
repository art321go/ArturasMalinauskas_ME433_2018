#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //math functions
#include<stdio.h>
#include "LED_HELP.h"      //given library for ST7735
#include "i2c_master_noint.h"    //library for I2C

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


int main () {
    __builtin_disable_interrupts();
    //setting up IMU for I2C communication and setting up LCD controls
    init_IMU();
    LCD_init();
        //TRIS and LAT commands
    TRISAbits.TRISA4 = 0b0;     //pin A4 is an output
    LATAbits.LATA4 = 0b1;       //A4 initially off
    __builtin_enable_interrupts(); 
    LCD_clearScreen(SECONDARY_COL);
    
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
    
    
    //drawing empty 2 wide crosshair at 64,80 , 50 in each direction ( 30 - 130 , 14 - 114)
    
    while (1){  
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
        
    
        //delay so the LED blinks at a perceptible rate that isnt annoying
        LedTime = LedTime + _CP0_GET_COUNT();
        if (LedTime > 5000000) { 
            LATAINV = 0b1 << 4;  //blinking the pic's LED to identify the code has not crashed 
            LedTime=0;
        }
        //
    }   
}
