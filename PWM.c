#include <xc.h>
#include "PWM.h"


void InitMCPWM(void) {
    IOCON1 = 0xAC00;//0x8c00;inv
    IOCON2 = 0xAC00;
    IOCON3 = 0xAC00;
    IOCON4 = 0xAC00;

    PTCON2 = 0x0000; // Divide by 1 to generate PWM

    PWMCON1 = 0x0004; // Enable PWM output pins and configure them as
    PWMCON2 = 0x0004; // complementary mode
    PWMCON3 = 0x0004;
    PWMCON4 = 0x0004;

    PTPER = 5000; //     60mips*2=  120mhz / 4000 = 30kz

    PDC1 = 0; // Initialize as 0 voltage
    PDC2 = 0; // 2;	// Initialize as 0 voltage
    PDC3 = 0; //PHASE3 / 2;	// Initialize as 0 voltage
    PDC4 = 0; //PHASE3 / 2;	// Initialize as 0 voltage

    PTCON = 0x8000; // start PWM

    return;
}