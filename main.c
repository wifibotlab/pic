/*
 * File:   main.c
 * Author: rbeal
 *
 * Created on May 5, 2017, 10:43 AM
 */

// DSPIC33EP512MC806 Configuration Bit Settings

// 'C' source line config statements

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF               // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Initial Oscillator Source Selection bits (Primary Oscillator (XT, HS, EC) with PLL)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = OFF             // PLL Lock Wait Enable bit (Clock switch will not wait for the PLL lock signal.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = OFF              // Brown-out Reset (BOR) Detection Enable bit (BOR is disabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF              // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF               // Auxiliary Segment Write-protect bit (Aux Flash may be written)
#pragma config APL = OFF                // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF               // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <dsp.h>
#include "global.h"
#include <libpic30.h>


#include "adc.h"
#include "rpn.h"
#include "IC.h"
#include "timers.h"
#include "PID.h"
#include "PWM.h"
#include "uart.h"
#include "vrac.h"
#include "I2C.h"
#include "sht21.h"
#include "mode_bus.h"


struct modbus modbus;

void init_pic(void);

//UART variables
unsigned char uart_start = 0;
unsigned char uart_sent = 0;
unsigned char uart_m_buf[20];
int uart_nchar = 0;

//Motors variable
unsigned short aux = 0;
unsigned short aux2 = 0;

//CRC 16 bits variables
short mycrc = 0;
short crcR = 0;
short crcS = 0;

// PID
unsigned char pp = 80, ii = 45, dd = 0; // coeff
tPID fooPID1;
tPID fooPID2;
tPID fooPID3;
tPID fooPID4;

unsigned char raspberry=0;

//wheels
int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
int speed4 = 0;

signed char speedl;
signed char speedr;

long position1 = 0;
long position2 = 0;
long position3 = 0;
long position4 = 0;

long positiontmp1 = 0;
long positiontmp2 = 0;
long positiontmp3 = 0;
long positiontmp4 = 0;

int dog = 0;
unsigned short maxspeed = 65000;
unsigned short speedmarge = 500;
char cmdflag = 0; //1 ubnt idle
char ch1 = 0;
char ch2 = 0;
char ch3 = 0;
char ch4 = 0;



int res1, res2, res3, res4;
int res, cmd, cmdd = 0, cmdg = 0, spd;


float qtmp1;
float qtmp2;



int myhallold1 = 0;
int myhallold2 = 0;
int myhallold3 = 0;
int myhallold4 = 0;

unsigned int HallValue1 = 0;
unsigned int HallValue2 = 0;
unsigned int HallValue3 = 0;
unsigned int HallValue4 = 0;

short bldcdir1;
short bldcdir2;
short bldcdir3;
short bldcdir4;

unsigned char bufposition1[4];
unsigned char bufposition2[4];
unsigned char bufposition3[4];
unsigned char bufposition4[4];

int main(void) {
    
    char ch1 = 0;
    char ch2 = 0;
    char ch3 = 0;
    char ch4 = 0;
    
    //ADC variables
    int tmpadc0 = 0;
    int tmpadc1 = 0;
    int tmpadc2 = 0;
    int tmpadc3 = 0;
    int tmpadc4 = 0;
    int tmpadc5 = 0;

    unsigned char capa_tmp=0;
    unsigned char volt_capa_tmp=0;
    unsigned char volt_capa=0;
    unsigned char capa_update=0;

    unsigned char tmp = 'a';

    short charge = 0;

    /*
    static unsigned char receivedMessage;
    static unsigned char slaveStatus;
    static unsigned char registerAddress;
    static unsigned char registerData;
    static unsigned char nextByte;
    static unsigned l;
    */

    int flip = 0, flipflop = 0;
    int count = 0, countrcv = 0;
    int pos = 0;
    int vittmp = 0;
    int vit = 0;
    short vitesse1 = 0;
    int AN_Sharp = 0;
    int AN_Sharp2 = 0;
    int AN_Bat = 0;
    int pos2 = 0;
    int vittmp2 = 0;
    int vit2 = 0;
    short vitesse2 = 0;

    long positionsend;
    int speedlab = 0;
    long positionsend2;
    int speedlab2 = 0;

    unsigned char ms10 = 0, msmode = 0;

    int cmd2, spd2;

    //Com buffer
    unsigned char bufrcv[5];
    unsigned char bufsend[19];

    int j=0;

    static double u[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
    static double y[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

    static double u2[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
    static double y2[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

    static double u3[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
    static double y3[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

    static double u4[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
    static double y4[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

    unsigned char cur_filtretmp=0;
    unsigned char cur_filtre=0;
    unsigned char cur_check_lim=0;

    unsigned char capafiltretmp=0;
    unsigned char capafiltre=0;
    
    double temp, hygro;
    
    init_pic();
    
    char str[40];
    
    int i, nSum;    
    
    while (1) {
        
        //X1-1 (IO4)
        LATBbits.LATB8 = 1;// raspberry pi On ch4;//ch1;  attention cabl\E9 a l'envers donc on inverse ici pour matcher le schema

        //X1-2 (io5)
        LATBbits.LATB9 = ch3;//ch2;

        //X1-3 (IO6)
        LATCbits.LATC14 = ch2;//ch3;..pb avec latb8 quand E0 ou B9 =1 ???? pb masse ?

        //X1-4 (IO7)
        LATEbits.LATE0 = ch1;//ch4;

        if (dog > modbus.wtd) {

            aux = 0x0000;
            aux2 = 0x0000;
            uart_nchar = 0;
            uart_start = 0;
            dog = 420; // ???
        }
        
        __delay_ms(10);

        tmpadc0 = ReadADC(0); //Av droite ir ok
        tmpadc1 = ReadADC(1); //ar gauche
        tmpadc2 = ReadADC(2); //v
        tmpadc4 = ReadADC(3); //av gauche ir ok
        tmpadc3 = ReadADC(4); //ar droite
        tmpadc5 = ReadADC(5); //i
        

        
        if (modbus.enable == 1) {
            
            { // PRE-CALC
                
                // CURRENT
                cur_filtretmp = (unsigned char)(tmpadc5 >> 2);
                cur_filtre = (unsigned char) (lp_filter2((unsigned char)(cur_filtretmp), u, y));
                cur_filtretmp = (unsigned char) (lp_filter2((unsigned char)(cur_filtre), u2, y2));
                
                // SHT21
                temp = get_SHT21_temp();
                hygro = get_SHT21_hygro();
                
            }
            
            { // STREAM
                
                bufsend[0] = MODBUS_SYNC_BYTE;
                bufsend[1] = 25;
                bufsend[2] = MODBUS_STREAM;
                
                bufsend[3] = bufposition1[0]; // odom avg
                bufsend[4] = bufposition1[1];
                bufsend[5] = bufposition1[2];
                bufsend[6] = bufposition1[3];
                
                bufsend[7]  = bufposition2[0]; // odom avd
                bufsend[8]  = bufposition2[1];
                bufsend[9]  = bufposition2[2];
                bufsend[10] = bufposition2[3];
               
                bufsend[11] = bufposition3[0]; // odom arg
                bufsend[12] = bufposition3[1];
                bufsend[13] = bufposition3[2];
                bufsend[14] = bufposition3[3];
                
                bufsend[15] = bufposition4[0]; // odom ard
                bufsend[16] = bufposition4[1];
                bufsend[17] = bufposition4[2];
                bufsend[18] = bufposition4[3];
                
                bufsend[19] = (unsigned char) (tmpadc2 *10 ); // voltage   // 1u *10
                bufsend[20] = (unsigned char) (cur_filtretmp *10); // current   // 1u *10
                bufsend[21] = (unsigned char) (temp  *2);  // temp
                bufsend[22] = (unsigned char) (hygro *2); // hygro
                
                bufsend[23] = speedl; // speed left  // 1s
                bufsend[24] = speedr; // speed right // 1s
                
                for (i=0;i<25;i++)
                    WriteUART1(bufsend[i]);
                
                mycrc = crc16(bufsend+1, 24);
                WriteUART1((unsigned char) mycrc);
                WriteUART1((unsigned char) (mycrc >> 8));
            }
            
            { // POST-CALC
                
                // CURRENT
                cur_filtretmp = (unsigned char)(((double)cur_filtretmp/13.0)*10.0);
                cur_check_lim = cur_filtretmp;
            }
                        
        }
        
        if (modbus.enable == 0) {
        
            bufsend[0] = (unsigned char) (speed3 * 5);
            bufsend[1] = (unsigned char) ((speed3 * 5) >> 8);
            bufsend[2] = (unsigned char) (tmpadc2 >> 2); //(unsigned char)(tmpadc2 >> 2);//10.1V 1.28V 404 /4 -> 101
            //bufsend[2]=bufsend[2]+1;
            //if (bufsend[2]==250) bufsend[2]=1;
            bufsend[3] = (unsigned char) (tmpadc4 >> 2); //3.3v->255 2v-> 624/4 -> 156
            bufsend[4] = (unsigned char) (tmpadc3 >> 2); //3.3v->255 2v-> 624/4 -> 156
            bufsend[5] = bufposition3[0];//3
            bufsend[6] = bufposition3[1];
            bufsend[7] = bufposition3[2];
            bufsend[8] = bufposition3[3];
            bufsend[9] = (unsigned char) (speed4 * 5);
            bufsend[10] = (unsigned char) ((speed4 * 5) >> 8);
            bufsend[11] = (unsigned char) (tmpadc0 >> 2);
            bufsend[12] = (unsigned char) (tmpadc1 >> 2);
            bufsend[13] = bufposition4[0];//4
            bufsend[14] = bufposition4[1];
            bufsend[15] = bufposition4[2];
            bufsend[16] = bufposition4[3];

            //bufsend[17] = (unsigned char) (tmpadc5 >> 2); //robot current

                ///////////////////////////
                //////////////////////////

            cur_filtretmp = (unsigned char)(tmpadc5 >> 2);

            cur_filtre = (unsigned char) (lp_filter2((unsigned char)(cur_filtretmp), u, y));
            cur_filtretmp = (unsigned char) (lp_filter2((unsigned char)(cur_filtre), u2, y2));

            bufsend[17]=(unsigned char)(cur_filtretmp);//robot current

            cur_filtretmp = (unsigned char)(((double)cur_filtretmp/13.0)*10.0);
            cur_check_lim = cur_filtretmp;

            ///////////////////////////
            //////////////////////////

            //Get Charge Status from charge board
            charge=!PORTBbits.RB7;

            //Green Led On / Off
            LATBbits.LATB6 = charge;

            volt_capa = bufsend[2];
            volt_capa_tmp = volt_capa;

            if(volt_capa<160)
            {
                if ((aux==0)&&(aux2==0))
                {

                    if (volt_capa>135)
                        volt_capa=135;

                    if (volt_capa>110)
                        capa_tmp=(volt_capa-110)*4;
                    else
                        capa_tmp=1;
                }

                capafiltretmp = (unsigned char) (lp_filter2((unsigned char)(capa_tmp), u3, y3));
                capafiltre = (unsigned char) (lp_filter2((unsigned char)(capafiltretmp), u4, y4));
            }
            else 
                capafiltre=101;

            bufsend[18] = capafiltre; //16 + charge; //firmware version PORTCbits.RC8;
            //bufsend[2]=capafiltre;

            //Debug
            //if (flipflop==0) {PORTBbits.RB3=1;flipflop=1;}
            //else {PORTBbits.RB3=0;flipflop=0;}
            //WriteUART1('a');

            if (raspberry==0) {
                WriteUART1(255);

                for (j = 0; j < 19; j++)
                    WriteUART1(bufsend[j]);

                //CRC 16 bits
                mycrc = crc16(bufsend, 19);
                WriteUART1((unsigned char) mycrc);
                WriteUART1((unsigned char) (mycrc >> 8));
            }
            else {
                WriteUART2(255);

                for (j = 0; j < 19; j++)
                    WriteUART2(bufsend[j]);

                //CRC 16 bits
                mycrc = crc16(bufsend, 19);
                WriteUART2((unsigned char) mycrc);
                WriteUART2((unsigned char) (mycrc >> 8));
            }
        }

    }
    
    return 0;
}

void init_pic(void) {
    
    unsigned int bootcount=0;
    
    //The settings below set up the oscillator and PLL for 60 MIPS as
    //follows:
    //            Crystal Frequency  * (DIVISOR+2)
    // Fcy =     ---------------------------------
    //              PLLPOST * (PRESCLR+2) * 4
    // Crystal  = 8 MHz
    // Fosc		= 120 MHz
    // Fcy		= 60 MIPS

    // Configure the oscillator to operate the device at 40 MHz
    // The Fast RC (FRC) internal oscillator runs at a nominal frequency of 7.37 MHz
    // FOSC = Fin * M/(N1 * N2), FCY= FOSC/2
    // FOSC = 7.37 * (43)/(2 * 2) = 80 MHz for FOSC, FCY= 40 MHz
    // In order to configure the device to operate at 40 MHz, configure the PLL prescaler,
    // PLL postscaler, and PLL divisor
    // PLLFBD = 41; // M = PLLFBD + 2
    // CLKDIVbits.PLLPOST = 0; // N1 = 2
    // CLKDIVbits.PLLPRE = 0; // N2 = 2

    PLLFBD = 58; //38=40mips; //58=60mips;// M=60
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    OSCTUN = 0;
    //PLLFBDbits.PLLDIV =38;
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);

    while (OSCCONbits.COSC != 0b011);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

    ACLKCON3 = 0x24C1;
    ACLKDIV3 = 0x7;
    ACLKCON3bits.ENAPLL = 1;
    while (ACLKCON3bits.APLLCK != 1);
    __delay_ms(10);
    ANSELB = 0x803F;//3F
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELG = 0x0000;

    //Power On GeneralTRISCbits
    TRISCbits.TRISC13 = 0;
    LATCbits.LATC13 = 0;

    TRISFbits.TRISF6 = 1; //3.3v old usb need to be input


    initAdc1();

    //re6
    //pwm and io power
    PORTE = 0x0000;
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;
    TRISEbits.TRISE4 = 0;
    TRISEbits.TRISE5 = 0;
    //TRISEbits.TRISE6 = 0;
    TRISEbits.TRISE7 = 0;
    //LATEbits.LATE6 = 1;

    //Pull Up
    //CNPU2bits.CN21PUE=1;
    //CNPU2bits.CN22PUE=1;
    //CNPU2bits.CN23PUE=1;

    //TRISGbits.TRISG6=1;
    //TRISFbits.TRISF5=0;

    //cmd
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;
    //TRISBbits.TRISB12 = 0;

    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;
    //TRISBbits.TRISB15 = 0;

    TRISDbits.TRISD6 = 0;
    TRISDbits.TRISD7 = 0;
    //TRISDbits.TRISD8 = 0;

    TRISDbits.TRISD9 = 0;
    TRISDbits.TRISD10 = 0;
    //TRISDbits.TRISD11 = 0;

    //coder
    TRISDbits.TRISD0 = 1; //A1
    TRISDbits.TRISD1 = 1; //B1
    TRISDbits.TRISD2 = 1; //C1

    TRISDbits.TRISD3 = 1; //A2
    TRISDbits.TRISD4 = 1; //B2
    TRISDbits.TRISD5 = 1; //C2

    TRISFbits.TRISF2 = 1; //A3  f2
    TRISFbits.TRISF5 = 1; //B3  f5
    TRISFbits.TRISF3 = 1; //C3  f3

    TRISFbits.TRISF0 = 1; //A4
    TRISFbits.TRISF1 = 1; //B4
    TRISFbits.TRISF4 = 1; //C4

    //channel 1 Ar Droite//pwm2
    LATBbits.LATB13 = 1;
    LATBbits.LATB14 = 0;
    //LATBbits.LATB15 = 0;

    //channel 2 Av Droite//pwm4
    LATDbits.LATD9 = 1;
    LATDbits.LATD10 = 0;
    //LATDbits.LATD11 = 0;

    //channel 3 Av Gauche //pwm3
    LATDbits.LATD6 = 1;
    LATDbits.LATD7 = 0;
    //LATDbits.LATD8 = 0;

    //channel 4   Ar Gauche //pwm1
    LATBbits.LATB10 = 1; //break
    LATBbits.LATB11 = 0; //dir
    //LATBbits.LATB12 = 0; //stop

    //0 to 8000
    PDC1 = 0;
    PDC2 = PDC1;
    PDC3 = PDC1;
    PDC4 = PDC1;

    initRpn();
    InitIC();
    Init_Timer2();
    initPID();
    Init_Timer1();
    InitMCPWM();
    init_I2C();
    init_SHT21();
    Init_Timer4(); // WTD

    //    InitUART1();
    //    InitUART2();

    //Power On General
    TRISCbits.TRISC13 = 0;
    LATCbits.LATC13 = 1;

    //Green Led
    TRISBbits.TRISB6 = 0;
    LATBbits.LATB6 = 0;

    //Charge status
    TRISBbits.TRISB7 = 1;
    //PORTBbits.RB7

    //IO Power
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB8 = 0;
    TRISEbits.TRISE0 = 0;
    TRISCbits.TRISC14 = 0;

    LATBbits.LATB8 = 1;//rasp
    
    for (bootcount=0;bootcount<160;bootcount++)
    {
        //__delay_ms(100);
    }
    InitUART1();
    InitUART2();
    
    modbus.enable = 1;
    
    modbus.speed_L = 0;
    modbus.speed_R = 0;
    
    modbus.wtd = 10;
    
    speedl = 0;
    speedr = 0;
}