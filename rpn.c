#include <xc.h>

#include <libpic30.h>
#include "rpn.h"

void initRpn() {
    //	__builtin_write_OSCCONL(OSCCON & 0xbf);    // clear bit 6

    RPINR7bits.IC1R = 64; //D0
    RPINR7bits.IC2R = 65; //D1
    RPINR8bits.IC3R = 66; //D2

    RPINR8bits.IC4R = 67; //D3
    RPINR9bits.IC5R = 68; //D4
    RPINR9bits.IC6R = 69; //D5

    RPINR10bits.IC7R = 98; //F
    RPINR10bits.IC8R = 101; //F
    RPINR33bits.IC9R = 99; //F

    RPINR33bits.IC10R = 96; //F0
    RPINR34bits.IC11R = 97; //F1
    RPINR34bits.IC12R = 100; //F4

    RPINR18bits.U1RXR = 118; //RX
    RPOR14bits.RP120R = 0b00001; //TX

    RPINR19bits.U2RXR = 86; //RX
    RPOR5bits.RP82R = 0b00011; //TX

    //	__builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS registers
}