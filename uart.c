#include <xc.h>
#include "uart.h"
#include "global.h"
#include <libpic30.h>
#include "vrac.h"
#include "PID.h"
#include "mode_bus.h"



extern unsigned char com_mode;

//UART variables
extern unsigned char uart_start;
extern unsigned char uart_sent;
extern unsigned char uart_m_buf[20];
extern int uart_nchar;

//CRC 16 bits variables
extern short mycrc;
extern short crcR;
extern short crcS;

//Motors variable
extern short aux;
extern short aux2;

// PID coeff
extern unsigned char pp, ii, dd;

extern unsigned char raspberry;

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


extern int dog;
extern short maxspeed;
extern short speedmarge;
extern char cmdflag; //1 ubnt idle
extern char ch1;
extern char ch2;
extern char ch3;
extern char ch4;

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    
    unsigned char ss;
    while(U1STAbits.URXDA)
    {
    ss = ReadUART1();

    if ((ss == MODBUS_SYNC_BYTE) && (uart_start == 0)) {

        uart_nchar = 1;
        uart_start = 1;
        modbus.enable = 1;
        uart_m_buf[0] = ss;
        ss = 0;
        

    } else if (uart_start == 1 && modbus.enable == 1) {
        
        uart_m_buf[uart_nchar] = ss;
        ss = 0;
        
        
        if (uart_nchar == (uart_m_buf[1]+1)) {
            
            
            
            crcR = crc16(uart_m_buf+1, uart_nchar-2);
            crcS = (short) uart_m_buf[uart_nchar-1] + (short) (uart_m_buf[uart_nchar] << 8);
            
            if (crcR == crcS) {
                
                // CRC OK !
                mode_bus(uart_m_buf);
                uart_start = 0;
                
                
            }
        }
        
        if (uart_nchar > (uart_m_buf[1]+1)) {
            
            uart_start = 0;
            // problem

        }
        
        uart_nchar++;
    }
    
    
    if ((ss == 255) && (uart_start == 0)) {
        
        uart_nchar = 0;
        uart_start = 1;
        modbus.enable = 0;

    } else if (uart_start == 1 && modbus.enable == 0) {

        uart_m_buf[uart_nchar] = ss;

        if ((uart_m_buf[0] == 7) || (uart_m_buf[0] == 9)) {

            if ((uart_nchar == 9) && (uart_m_buf[0] == 9)) {

                crcR = crc16(uart_m_buf, 8);

                crcS = (short) (uart_m_buf[9] << 8) + (short) uart_m_buf[8];

                crcR = crc16(uart_m_buf, 8);

                if (crcR == crcS) {

                    aux = uart_m_buf[1];

                    aux2 = uart_m_buf[2];

                    pp = uart_m_buf[3];

                    ii = uart_m_buf[4];

                    dd = uart_m_buf[5];

                    //maxspeed = ((short) (uart_m_buf[7] << 8) + uart_m_buf[6]);
                    raspberry=0;
                    speed1=0;
                    position1=0;
                    positiontmp1=0;

                    speed2=0;
                    position2=0;
                    positiontmp2=0;

                    speed3=0;
                    position3=0;
                    positiontmp3=0;

                    speed4=0;
                    position4=0;
                    positiontmp4=0;
                    initPID();
                    }

                uart_start = 0;
                uart_nchar = 0;

            } else if ((uart_nchar == 7) && (uart_m_buf[0] == 7)) {

                crcR = crc16(uart_m_buf, 6);

                crcS = (short) (uart_m_buf[7] << 8) + (short) uart_m_buf[6];

                if (crcR == crcS) {

                    dog = 0;
                    
                    aux = (short) (uart_m_buf[2] << 8) + (short) uart_m_buf[1];

                    aux2 = (short) (uart_m_buf[4] << 8) + (short) uart_m_buf[3];

                    if (aux > (maxspeed - speedmarge)) aux = maxspeed - speedmarge;

                    if (aux2 > (maxspeed - speedmarge)) aux2 = maxspeed - speedmarge;

                    cmdflag = uart_m_buf[5];

                    if(cmdflag&0x01) ch1=1;
                    else if(!(cmdflag&0x01)) ch1=0;

                    if(cmdflag&0x02) ch2=1;
                    else if(!(cmdflag&0x02)) ch2=0;

                    if(cmdflag&0x04) ch3=1;
                    else if(!(cmdflag&0x04)) ch3=0;

                    if(cmdflag&0x08) ch4=1;
                    else if(!(cmdflag&0x08)) ch4=0;
                }

                uart_start = 0;
                uart_nchar = 0;
            }/////////////////////////////////////////////////////////////////////////////////////////////////////////////
            else if ((uart_nchar == 4) && (uart_m_buf[0] == 4)) {

                crcR = crc16(uart_m_buf, 3);

                crcS = (short) (uart_m_buf[4] << 8) + (short) uart_m_buf[3];

                if (crcR == crcS) {

                    dog = 0;

                    //do proc data
                }
                uart_start = 0;
                uart_nchar = 0;
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        } else {

            uart_start = 0;

            uart_nchar = 0;

        }
        uart_nchar++;
    }
     }
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    //WriteUART1((unsigned char)69);
    IFS0bits.U1TXIF = 0;
    //U1TXREG = 'z';
}

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {

    unsigned char ss;
      while(U2STAbits.URXDA)
   {
    ss = ReadUART2();
    if ((ss == 255) && (uart_start == 0)) {

        uart_nchar = 0;

        uart_start = 1;

    } else if (uart_start == 1) {

        uart_m_buf[uart_nchar] = ss;

        if ((uart_m_buf[0] == 7) || (uart_m_buf[0] == 9)) {

            if ((uart_nchar == 9) && (uart_m_buf[0] == 9)) {

                crcR = crc16(uart_m_buf, 8);

                crcS = (short) (uart_m_buf[9] << 8) + (short) uart_m_buf[8];

                if (crcR == crcS) {
                   
                    aux = uart_m_buf[1];

                    aux2 = uart_m_buf[2];

                    pp = uart_m_buf[3];

                    ii = uart_m_buf[4];

                    dd = uart_m_buf[5];

                    //maxspeed = ((short) (uart_m_buf[7] << 8) + uart_m_buf[6]);
                    raspberry=1;
                    speed1=0;
                    position1=0;
                    positiontmp1=0;

                    speed2=0;
                    position2=0;
                    positiontmp2=0;

                    speed3=0;
                    position3=0;
                    positiontmp3=0;

                    speed4=0;
                    position4=0;
                    positiontmp4=0;
                    initPID();
                    }

                uart_start = 0;
                uart_nchar = 0;

            } else if ((uart_nchar == 7) && (uart_m_buf[0] == 7)) {

                crcR = crc16(uart_m_buf, 6);

                crcS = (short) (uart_m_buf[7] << 8) + (short) uart_m_buf[6];

                if (crcR == crcS) {

                    dog = 0;
                   
                    aux = (short) (uart_m_buf[2] << 8) + (short) uart_m_buf[1];

                    aux2 = (short) (uart_m_buf[4] << 8) + (short) uart_m_buf[3];

                    if (aux > (maxspeed - speedmarge)) aux = maxspeed - speedmarge;

                    if (aux2 > (maxspeed - speedmarge)) aux2 = maxspeed - speedmarge;

                    cmdflag = uart_m_buf[5];

                    if(cmdflag&0x01) ch1=1;
                    else if(!(cmdflag&0x01)) ch1=0;

                    if(cmdflag&0x02) ch2=1;
                    else if(!(cmdflag&0x02)) ch2=0;

                    if(cmdflag&0x04) ch3=1;
                    else if(!(cmdflag&0x04)) ch3=0;

                    if(cmdflag&0x08) ch4=1;
                    else if(!(cmdflag&0x08)) ch4=0;
                }

                uart_start = 0;
                uart_nchar = 0;
            }/////////////////////////////////////////////////////////////////////////////////////////////////////////////
            else if ((uart_nchar == 4) && (uart_m_buf[0] == 4)) {

                crcR = crc16(uart_m_buf, 3);

                crcS = (short) (uart_m_buf[4] << 8) + (short) uart_m_buf[3];

                if (crcR == crcS) {

                    dog = 0;

                    //do proc data
                }
                uart_start = 0;
                uart_nchar = 0;
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        } else {

            uart_start = 0;

            uart_nchar = 0;

        }
        uart_nchar++;
    }
   }
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    //WriteUART1((unsigned char)69);
    IFS1bits.U2TXIF = 0;
    //U1TXREG = 'z';
}


void InitUART1() {
    // The HPC16 board has a DB9 connector wired to UART2, so we will
    // be configuring this port only
    // configure U2MODE
    U1MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U2MODEbits.notimplemented;	// Bit14
    U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U1MODEbits.IREN = 0; // Bit12 No IR translation
    U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U2MODEbits.notimplemented;	// Bit10
    U1MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
    U1MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U1MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U1MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
    U1MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0; // Bit0 One Stop Bit

    // Load a value into Baud Rate Generator.  Example is for 9600.
    // See section 19.3.1 of datasheet.
    //  U2BRG = (Fcy/(16*BaudRate))-1
    //  U2BRG = (37M/(16*9600))-1
    //  U2BRG = 240
    U1BRG = 194; //194	;// 40Mhz osc, 9600 Baud

    U1MODEbits.URXINV = 0;

    U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    // Load all values in for U1STA SFR
    U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U1STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
    //U2STAbits.notimplemented = 0;	//Bit12
    U1STAbits.UTXBRK = 0; //Bit11 Disabled
    U1STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph //L
    U1STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U1STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U1STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
    U1STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U1STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U1STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U1STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U1STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U1STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    //	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

    IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 0; // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts
    U1MODEbits.UARTEN = 1; // And turn the peripheral on
    U1STAbits.UTXEN = 1;
}

void InitUART2() {
    // The HPC16 board has a DB9 connector wired to UART2, so we will
    // be configuring this port only
    // configure U2MODE
    U2MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U2MODEbits.notimplemented;	// Bit14
    U2MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U2MODEbits.IREN = 0; // Bit12 No IR translation
    U2MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U2MODEbits.notimplemented;	// Bit10
    U2MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
    U2MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U2MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U2MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')

    U2MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U2MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U2MODEbits.STSEL = 0; // Bit0 One Stop Bit

    // Load a value into Baud Rate Generator.  Example is for 9600.
    // See section 19.3.1 of datasheet.
    //  U2BRG = (Fcy/(16*BaudRate))-1
    //  U2BRG = (37M/(16*9600))-1
    //  U2BRG = 240
    U2BRG = 194; //194	;// 40Mhz osc, 9600 Baud

    U2MODEbits.URXINV = 0;

    U2STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    // Load all values in for U1STA SFR
    U2STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U2STAbits.UTXINV = 0; //Bit14 N/A, IRDA config

    //U2STAbits.notimplemented = 0;	//Bit12
    U2STAbits.UTXBRK = 0; //Bit11 Disabled
    U2STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph //L
    U2STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U2STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U2STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
    U2STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U2STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U2STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U2STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U2STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U2STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    //	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

    IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 0; // Enable Transmit Interrupts
    IFS1bits.U2RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC1bits.U2RXIE = 1; // Enable Recieve Interrupts
    U2MODEbits.UARTEN = 1; // And turn the peripheral on
    U2STAbits.UTXEN = 1;
}

void WriteUART1(unsigned char c) {
    
    U1TXREG = c;
    while(U1STAbits.UTXBF);
    //__delay_ms(1);
}

void UART_str(char *str) {
    int i;
    
    for (i=0;*(str+i) != '\0';i++)
        WriteUART1(*(str+i));
}

void WriteUART2(unsigned char c) {
    
    U2TXREG = c;
    __delay_us(600);
}

unsigned char ReadUART1(void) {
    
    return U1RXREG;
}

unsigned char ReadUART2(void) {
    
    return U2RXREG;
}