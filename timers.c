#include <xc.h>
#include <dsp.h>
#include "timers.h"
#include "mode_bus.h"
#include "global.h"

// PID
extern tPID fooPID1;
extern tPID fooPID2;
extern tPID fooPID3;
extern tPID fooPID4;

//wheels
extern int speed1;
extern int speed2;
extern int speed3;
extern int speed4;

extern long position1;
extern long position2;
extern long position3;
extern long position4;

extern long positiontmp1;
extern long positiontmp2;
extern long positiontmp3;
extern long positiontmp4;


extern int res1, res2, res3, res4;
extern int res, cmd, cmdd, cmdg, spd;

//Motors variable
extern unsigned short aux;
extern unsigned short aux2;

extern float qtmp1;
extern float qtmp2;

extern char cmdflag; //1 ubnt idle

extern int dog;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;

   // LATEbits.LATE6 = !LATEbits.LATE6;

    cmdg = aux;
    cmdd = aux2;

    //aux seems to be from 0 to 340 ??
    if ( ! modbus.enable) {
        if (cmdg > 250) cmdg = 250; // mode 8b
        if (cmdd > 250) cmdd = 250; // mode 8b
    }

    if (!(cmdflag & 0x40)) cmdg = -cmdg;
    if (!(cmdflag & 0x10)) cmdd = -cmdd;

    if (!(cmdflag & 0x80)) {   // no PID
        	PIDInit(&fooPID1);
            PIDInit(&fooPID2);
            PIDInit(&fooPID3);
            PIDInit(&fooPID4);
        if (aux != 0) {
            //channel 3 Av Gauche
            LATDbits.LATD6 = 1;
            //LATDbits.LATD8 = 0;
            //channel 4   Ar Gauche
            LATBbits.LATB10 = 1; //break
            //LATBbits.LATB12 = 0; //stop
        } else {
            //channel 3 Av Gauche
            //LATDbits.LATD8 = 0;
            //LATBbits.LATB12 = 0; //stop

            LATDbits.LATD6 = 0;
            LATBbits.LATB10 = 0; //break
        }

        if (aux2 != 0) {
            //channel 2 Av Droite
            LATDbits.LATD9 = 1;
            //LATDbits.LATD11 = 0;

            //channel 1 Ar Droite
            LATBbits.LATB13 = 1;
            //LATBbits.LATB15 = 0;
        } else {
            //channel 1 Ar Droite
            //LATBbits.LATB15 = 0;
            //channel 2 Av Droite
            //LATDbits.LATD11 = 0;

            LATDbits.LATD9 = 0;
            LATBbits.LATB13 = 0;
        }

        if (cmdflag & 0x10)//set direction
        {
            LATBbits.LATB14 = 0;
            LATDbits.LATD10 = 0;
        } else {
            LATBbits.LATB14 = 1;
            LATDbits.LATD10 = 1;
        }

        if (cmdflag & 0x40)//set direction
        {
            LATBbits.LATB11 = 1;
            LATDbits.LATD7 = 1;
        } else {
            LATBbits.LATB11 = 0;
            LATDbits.LATD7 = 0;
        }

        //0 to 5000
        //Ar Gauche
        if ( ! modbus.enable) {
            PDC1 = aux * 20;
        }
        else {
            PDC1 = aux * 0.0763;
        }
        //PDC1 = (short) ((float)aux * 0.0763); // mode 16b
        //Av Gauche
        PDC3 = PDC1;

        //Ar Droite
        if ( ! modbus.enable) {
            PDC2 = aux2 * 20;
        }
        else {
            PDC2 = aux2 * 0.0763;
        }
        //PDC2 = (short) ((float)aux2 * 0.0763); // mode 16b
        //Av Droite
        PDC4 = PDC2;
    } else {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Gauche
        //Ar
       if (cmdg!=0) {
           
        if ( ! modbus.enable) {
            qtmp1 = (float) ((float) cmdd / (float) 250); // mode 8b
        }
        else {
            qtmp1 = (float) ((float) cmdd / (float) 65500);
        }
        
        qtmp2 = (float) ((float) speed3 / (float) 100);
        fooPID1.controlReference = Q15(qtmp1);
        fooPID1.measuredOutput = Q15(qtmp2);
        PID(&fooPID1);
        res3 = Fract2Float(fooPID1.controlOutput)*5000; //max speed 60 tics at pwm 5000
        //Av
        if ( ! modbus.enable) {
            qtmp1 = (float) ((float) cmdg / (float) 250); // mode 8b
        }
        else {
            qtmp1 = (float) ((float) cmdg / (float) 65500);
        }
        qtmp2 = (float) ((float) speed1 / (float) 100);
        fooPID2.controlReference = Q15(qtmp1);
        fooPID2.measuredOutput = Q15(qtmp2);
        PID(&fooPID2);
        res1 = Fract2Float(fooPID2.controlOutput)*5000; //max speed 60 tics at pwm 5000
       
        //channel 3 Av Gauche
        LATDbits.LATD6 = 1;
        //LATDbits.LATD8 = 0;

        //channel 4   Ar Gauche
        LATBbits.LATB10 = 1; //break
        //LATBbits.LATB12 = 0; //stop

        if (res3 >= 0) LATDbits.LATD7 = 1;
        else LATDbits.LATD7 = 0;

        if (res1 >= 0) LATBbits.LATB11 = 1;
        else LATBbits.LATB11 = 0;
       }
       else {
       //LATDbits.LATD8 = 0;
       //LATBbits.LATB12 = 0; //stop
       LATBbits.LATB10 = 0; //break
       LATDbits.LATD6 = 0;
       PIDInit(&fooPID1);
       PIDInit(&fooPID2);
       res3=0.0;
       res1=0.0;
       }

       if (cmdd!=0) {
	//Droit
        //Ar
           
        if ( ! modbus.enable) {
            qtmp1 = (float) ((float) cmdd / (float) 250); // mode 8b
        }
        else {
            qtmp1 = (float) ((float) cmdd / (float) 65500);
        }
        qtmp2 = (float) ((float) speed2 / (float) 100);
        fooPID3.controlReference = Q15(qtmp1);
        fooPID3.measuredOutput = Q15(qtmp2);
        PID(&fooPID3);
        res2 = Fract2Float(fooPID3.controlOutput)*5000; //max speed 60 tics at pwm 5000
        //Av
        if ( ! modbus.enable) {
            qtmp1 = (float) ((float) cmdg / (float) 250); // mode 8b
        }
        else {
            qtmp1 = (float) ((float) cmdg / (float) 65500);
        }
        qtmp2 = (float) ((float) speed4 / (float) 100);
        fooPID4.controlReference = Q15(qtmp1);
        fooPID4.measuredOutput = Q15(qtmp2);
        PID(&fooPID4);
        res4 = Fract2Float(fooPID4.controlOutput)*5000; //max speed 60 tics at pwm 5000
       
        //channel 1 Ar Droite
        LATBbits.LATB13 = 1;
       // LATBbits.LATB15 = 0;

        //channel 2 Av Droite
        LATDbits.LATD9 = 1;
        //LATDbits.LATD11 = 0;

        if (res2 >= 0) LATBbits.LATB14 = 0;
        else LATBbits.LATB14 = 1;

        if (res4 >= 0) LATDbits.LATD10 = 0;
        else LATDbits.LATD10 = 1;

        }
       else {
        //LATBbits.LATB15 = 0;
        //LATDbits.LATD11 = 0;
        LATDbits.LATD9 = 0;
        LATBbits.LATB13 = 0;

       PIDInit(&fooPID3);
       PIDInit(&fooPID4);
       res2=0.0;
       res4=0.0;
       }
       

        //0 to 8000
        //Ar Gauche
        PDC1 = abs(res1);
        //Av Gauche
        PDC3 = abs(res3);

        //Ar Droite
        PDC2 = abs(res2);
        //Av Droite
        PDC4 = abs(res4);
        
    }//end else
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;

    speed1 = position1 - positiontmp1;
    positiontmp1 = position1;

    speed2 = position2 - positiontmp2;
    positiontmp2 = position2;

    speed3 = position3 - positiontmp3;
    positiontmp3 = position3;

    speed4 = position4 - positiontmp4;
    positiontmp4 = position4;
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    
    dog++;
    IFS1bits.T4IF = 0;
}

void Init_Timer1(void) {

    T1CON = 0x30; // Timer reset //no divider
    IFS0bits.T1IF = 0; // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 6; // Timer1 Interrupt priority level=4
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    TMR1 = 0x0000;
    PR1 = 2400; // 2400 Timer1 period register = 10 mS a 60 MIPS
    T1CONbits.TON = 1; // Enable Timer1 and start the counter
}

void Init_Timer2(void) {

    T2CON = 0x30; // Timer reset /256
    IFS0bits.T2IF = 0; // Reset Timer1 interrupt flag
    IPC1bits.T2IP = 6; // Timer1 Interrupt priority level=4
    IEC0bits.T2IE = 1; // Enable Timer1 interrupt
    TMR2 = 0x0000;
    PR2 = 12000; // 12000 Timer1 period register = 51.2 mS at 60 MIPS
    T2CONbits.TON = 1; // Enable Timer1 and start the counter
}

void Init_Timer3(void) {

    T3CON = 0; // Timer reset //no divider
    IFS0bits.T3IF = 0; // Reset Timer1 interrupt flag
    IPC2bits.T3IP = 6; // Timer1 Interrupt priority level=4
    IEC0bits.T3IE = 1; // Enable Timer1 interrupt
    TMR3 = 0x0000;
    PR3 = 6000; // 6000 Timer1 period register = 0.1 mS a 60 MIPS for 40mips 4000
    T3CONbits.TON = 1; // Enable Timer1 and start the counter
}

void Init_Timer4(void) {

    T4CON = 0; // Timer reset //no divider
    T4CONbits.TCKPS = 0b11; // /256
    IFS1bits.T4IF = 0; // Reset Timer1 interrupt flag
    IPC6bits.T4IP = 6; // Timer1 Interrupt priority level=4
    IEC1bits.T4IE = 1; // Enable Timer1 interrupt
    TMR4 = 0x0000;
    PR4 = 2343; // period = 10ms
    T4CONbits.TON = 1; // Enable Timer1 and start the counter
}
