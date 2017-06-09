#include <xc.h>
#include "global.h"
#include <libpic30.h>
#include "adc.h"

void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt (void)
{
	IFS0bits.AD1IF = 0;		// Clear interrupt flag
	return;
}


void initAdc1(void) {
    /* Initialize and enable ADC module */
    AD1CON1 = 0x000C; // Enable simultaneous sampling and auto-sample
    AD1CON2 = 0x0000; // Sample 4 channels 0x0300
    AD1CON3 = 0x0005; //0x000f
    AD1CON4 = 0x0000;
    AD1CSSH = 0x0000;
    AD1CSSL = 0x0000;
    AD1CHS123bits.CH123SA = 0; // Select AN0 for CH1 +ve input
    // Select AN1 for CH2 +ve input
    // Select AN2 for CH3 +ve input
    AD1CHS123bits.CH123NA = 0; // Select Vref- for CH1/CH2/CH3 -ve inputs
    AD1CON1bits.ADON = 1;
    __delay_us(20);
}

int ReadADC(int ch) {

    AD1CHS0bits.CH0SA = ch; //Select Analog Input
    AD1CHS0bits.CH0NA = 0; //Select Analog Input
    AD1CON1bits.SAMP = 1; //start sampling
    __delay_us(20); //see data sheet
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE); //wait to complete convertion
    return ADC1BUF0; //read result
}