#include <xc.h>
#include <libpic30.h>
#include "IC.h"
#include "vrac.h"

extern int myhallold1;
extern int myhallold2;
extern int myhallold3;
extern int myhallold4;

extern unsigned int HallValue1;
extern unsigned int HallValue2;
extern unsigned int HallValue3;
extern unsigned int HallValue4;

extern short bldcdir1;
extern short bldcdir2;
extern short bldcdir3;
extern short bldcdir4;

extern long position1;
extern long position2;
extern long position3;
extern long position4;

extern unsigned char bufposition1[4];
extern unsigned char bufposition2[4];
extern unsigned char bufposition3[4];
extern unsigned char bufposition4[4];

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void) {
    IFS0bits.IC1IF = 0; // Clear interrupt flag

    HallValue1 = (unsigned int) ((PORTD) & 0x0007); // Read halls
    bldcdir1 = odometer(HallValue1, myhallold1);
    if (bldcdir1 == 0) position1--;
    else if (bldcdir1 == 1) position1++;
    myhallold1 = HallValue1;

    bufposition1[0] = position1;
    bufposition1[1] = position1 >> 8;
    bufposition1[2] = position1 >> 16;
    bufposition1[3] = position1 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void) {
    IFS0bits.IC2IF = 0; // Clear interrupt flag
    HallValue1 = (unsigned int) ((PORTD) & 0x0007); // Read halls
    bldcdir1 = odometer(HallValue1, myhallold1);
    if (bldcdir1 == 0) position1--;
    else if (bldcdir1 == 1) position1++;
    myhallold1 = HallValue1;

    bufposition1[0] = position1;
    bufposition1[1] = position1 >> 8;
    bufposition1[2] = position1 >> 16;
    bufposition1[3] = position1 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt(void) {
    IFS2bits.IC3IF = 0; // Clear interrupt flag
    HallValue1 = (unsigned int) ((PORTD) & 0x0007); // Read halls
    bldcdir1 = odometer(HallValue1, myhallold1);
    if (bldcdir1 == 0) position1--;
    else if (bldcdir1 == 1) position1++;
    myhallold1 = HallValue1;

    bufposition1[0] = position1;
    bufposition1[1] = position1 >> 8;
    bufposition1[2] = position1 >> 16;
    bufposition1[3] = position1 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC4Interrupt(void) {
    IFS2bits.IC4IF = 0; // Clear interrupt flag

    HallValue2 = (unsigned int) ((PORTD >> 3) & 0x0007); // Read halls
    bldcdir2 = odometer(HallValue2, myhallold2);
    if (bldcdir2 == 1) position2--;
    else if (bldcdir2 == 0) position2++;
    myhallold2 = HallValue2;

    bufposition2[0] = position2;
    bufposition2[1] = position2 >> 8;
    bufposition2[2] = position2 >> 16;
    bufposition2[3] = position2 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC5Interrupt(void) {
    IFS2bits.IC5IF = 0; // Clear interrupt flag
    HallValue2 = (unsigned int) ((PORTD >> 3) & 0x0007); // Read halls
    bldcdir2 = odometer(HallValue2, myhallold2);
    if (bldcdir2 == 1) position2--;
    else if (bldcdir2 == 0) position2++;
    myhallold2 = HallValue2;

    bufposition2[0] = position2;
    bufposition2[1] = position2 >> 8;
    bufposition2[2] = position2 >> 16;
    bufposition2[3] = position2 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC6Interrupt(void) {
    IFS2bits.IC6IF = 0; // Clear interrupt flag
    HallValue2 = (unsigned int) ((PORTD >> 3) & 0x0007); // Read halls
    bldcdir2 = odometer(HallValue2, myhallold2);
    if (bldcdir2 == 1) position2--;
    else if (bldcdir2 == 0) position2++;
    myhallold2 = HallValue2;

    bufposition2[0] = position2;
    bufposition2[1] = position2 >> 8;
    bufposition2[2] = position2 >> 16;
    bufposition2[3] = position2 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt(void) {
    IFS1bits.IC7IF = 0; // Clear interrupt flag
    //HallValue3 = ((unsigned int) ((PORTF >> 2) & 0x0003))+(((unsigned int) (PORTFbits.RF5)) << 2); // Read halls//(unsigned int) ((PORTG >> 7) & 0x0007);
     HallValue3 = (unsigned int)(((unsigned int) ((PORTF >> 2) & 0x0001)) + (((unsigned int) (PORTFbits.RF5)) << 1) + (((unsigned int) (PORTFbits.RF3)) << 2));
    bldcdir3 = odometer(HallValue3, myhallold3);
    if (bldcdir3 == 0) position3--;
    else if (bldcdir3 == 1) position3++;
    myhallold3 = HallValue3;

    bufposition3[0] = position3;
    bufposition3[1] = position3 >> 8;
    bufposition3[2] = position3 >> 16;
    bufposition3[3] = position3 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC8Interrupt(void) {
    IFS1bits.IC8IF = 0; // Clear int((PORTF >> 2) & 0x0003))+(((unsigned int) (PORTFbits.RF5)) << 2); // Read halls//(unsigned int) ((PORTG >> 7) & 0x0007);
    HallValue3 = (unsigned int)(((unsigned int) ((PORTF >> 2) & 0x0001)) + (((unsigned int) (PORTFbits.RF5)) << 1) + (((unsigned int) (PORTFbits.RF3)) << 2));
    bldcdir3 = odometer(HallValue3, myhallold3);
    if (bldcdir3 == 0) position3--;
    else if (bldcdir3 == 1) position3++;
    myhallold3 = HallValue3;

    bufposition3[0] = position3;
    bufposition3[1] = position3 >> 8;
    bufposition3[2] = position3 >> 16;
    bufposition3[3] = position3 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC9Interrupt(void) {
    IFS5bits.IC9IF = 0; // Clear interrupt flag
    //HallValue3 = ((unsigned int) ((PORTF >> 2) & 0x0003))+(((unsigned int) (PORTFbits.RF5)) << 2); // Read halls//(unsigned int) ((PORTG >> 7) & 0x0007);
    HallValue3 = (unsigned int)(((unsigned int) ((PORTF >> 2) & 0x0001)) + (((unsigned int) (PORTFbits.RF5)) << 1) + (((unsigned int) (PORTFbits.RF3)) << 2));
    //HallValue3 = ((unsigned int) 
    bldcdir3 = odometer(HallValue3, myhallold3);
    if (bldcdir3 == 0) position3--;
    else if (bldcdir3 == 1) position3++;
    myhallold3 = HallValue3;

    bufposition3[0] = position3;
    bufposition3[1] = position3 >> 8;
    bufposition3[2] = position3 >> 16;
    bufposition3[3] = position3 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC10Interrupt(void) {
    IFS7bits.IC10IF = 0; // Clear interrupt flag
    HallValue4 = ((unsigned int) ((PORTF) & 0x0003))+(((unsigned int) (PORTFbits.RF4)) << 2); // Read halls
    bldcdir4 = odometer(HallValue4, myhallold4);
    if (bldcdir4 == 1) position4--;
    else if (bldcdir4 == 0) position4++;
    myhallold4 = HallValue4;

    bufposition4[0] = position4;
    bufposition4[1] = position4 >> 8;
    bufposition4[2] = position4 >> 16;
    bufposition4[3] = position4 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC11Interrupt(void) {
    IFS7bits.IC11IF = 0; // Clear interrupt flag
    HallValue4 = ((unsigned int) ((PORTF) & 0x0003))+(((unsigned int) (PORTFbits.RF4)) << 2); // Read halls
    bldcdir4 = odometer(HallValue4, myhallold4);
    if (bldcdir4 == 1) position4--;
    else if (bldcdir4 == 0) position4++;
    myhallold4 = HallValue4;

    bufposition4[0] = position4;
    bufposition4[1] = position4 >> 8;
    bufposition4[2] = position4 >> 16;
    bufposition4[3] = position4 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _IC12Interrupt(void) {
    IFS8bits.IC12IF = 0; // Clear interrupt flag
    HallValue4 = ((unsigned int) ((PORTF) & 0x0003))+(((unsigned int) (PORTFbits.RF4)) << 2); // Read halls
    bldcdir4 = odometer(HallValue4, myhallold4);
    if (bldcdir4 == 1) position4--;
    else if (bldcdir4 == 0) position4++;
    myhallold4 = HallValue4;

    bufposition4[0] = position4;
    bufposition4[1] = position4 >> 8;
    bufposition4[2] = position4 >> 16;
    bufposition4[3] = position4 >> 24;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    IFS1bits.CNIF = 0;
}

void InitIC(void) {

    IC1CON1 = 1; // Init all 3 Hall Effect capture inputs:
    IC2CON1 = 1; // Timer 3, every capture event, rise & fall edges
    IC3CON1 = 1;
    IC4CON1 = 1;
    IC5CON1 = 1;
    IC6CON1 = 1;
    IC7CON1 = 1;
    IC8CON1 = 1;
    IC9CON1 = 1;
    IC10CON1 = 1;
    IC11CON1 = 1;
    IC12CON1 = 1;

    IFS0bits.IC1IF = 0; // Clear interrupt flag
    IFS0bits.IC2IF = 0; // Clear interrupt flag
    IFS2bits.IC3IF = 0; // Clear interrupt flag
    IFS2bits.IC4IF = 0; // Clear interrupt flag
    IFS2bits.IC5IF = 0; // Clear interrupt flag
    IFS2bits.IC6IF = 0; // Clear interrupt flag
    IFS1bits.IC7IF = 0; // Clear interrupt flag
    IFS1bits.IC8IF = 0; // Clear interrupt flag
    IFS5bits.IC9IF = 0; // Clear interrupt flag
    IFS7bits.IC10IF = 0; // Clear interrupt flag
    IFS7bits.IC11IF = 0; // Clear interrupt flag
    IFS8bits.IC12IF = 0; // Clear interrupt flag

    IEC0bits.IC1IE = 1; // Enable interrupt
    IEC0bits.IC2IE = 1;
    IEC2bits.IC3IE = 1;
    IEC2bits.IC4IE = 1;
    IEC2bits.IC5IE = 1;
    IEC2bits.IC6IE = 1;
    IEC1bits.IC7IE = 1;
    IEC1bits.IC8IE = 1;
    IEC5bits.IC9IE = 1;
    IEC7bits.IC10IE = 1;
    IEC7bits.IC11IE = 1;
    IEC8bits.IC12IE = 1;
}
