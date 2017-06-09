#include <xc.h>
#include "global.h"
#include <libpic30.h>
#include "I2C.h"

void init_I2C(void) {
    
    I2C1CONbits.I2CEN = 1; // enable I2C + configure pin
    I2C1BRG = 590; // 100k
}

void I2C_write_byte(unsigned char device_addr, unsigned char addr, unsigned char data)
{
    I2C1CONbits.SEN = 1; // start bit
    I2C_busy();
    
    I2C1TRN = device_addr & 0xFE; // write mode
    I2C_busy();
    wait_ack();
    
    I2C1TRN = addr; // addr
    I2C_busy();
    wait_ack();
    
    I2C1TRN = data; // data
    I2C_busy();
    wait_ack();
    
    I2C1CONbits.PEN = 1; // stop bit
    I2C_busy();
}

char I2C_read_byte(unsigned char device_addr, unsigned char addr)
{
    char tmp;
    
    I2C1CONbits.SEN = 1; // start bit
    I2C_busy();
    
    I2C1TRN = device_addr & 0xFE; // write mode
    I2C_busy();
    wait_ack();
    
    I2C1TRN = addr; // addr
    I2C_busy();
    wait_ack();
    
    I2C1CONbits.PEN = 1; // stop bit
    I2C_busy();
    
    I2C1CONbits.RSEN = 1; // Restart bit
    I2C_busy();
    
    
    I2C1TRN = device_addr | 0x01; // read mode
    I2C_busy();
    wait_ack();
    
    I2C1CONbits.RCEN = 1;
    I2C_busy();
    
    tmp = I2C1RCV;
    I2C1CONbits.ACKDT = 0;
    
    I2C1CONbits.PEN = 1; // stop bit
    I2C_busy();
    
return tmp;
}

void I2C_read_multibytes(unsigned char device_addr, unsigned char addr, unsigned char* data, unsigned char nb_bytes)
{
    I2C1CONbits.SEN = 1; // start bit
    I2C_busy();
    
    I2C1TRN = device_addr & 0xFE; // write mode
    I2C_busy();
    wait_ack();
    
    I2C1TRN = addr; // addr
    I2C_busy();
    wait_ack();
    
    I2C1CONbits.PEN = 1; // stop bit
    I2C_busy();
    
    I2C1CONbits.RSEN = 1; // Restart bit
    I2C_busy();
    
    I2C1TRN = device_addr | 0x01; // read mode
    I2C_busy();
    wait_ack();
    
    unsigned char i;
    for(i = 0; i < nb_bytes; i++) {
        
        I2C1CONbits.RCEN = 1;
        I2C_busy();

        *(data + i) = I2C1RCV;
        I2C1CONbits.ACKDT = (i < nb_bytes - 1) ? 0 : 1;
    }
    
    I2C1CONbits.PEN = 1; // stop bit
    I2C_busy();
}

void I2C_busy()
{
    while((I2C1CON & 0x001F) || (I2C1STAT & 0x4000));
}

void wait_ack()
{
    int timeout=50000;
    
    while (I2C1STATbits.ACKSTAT && timeout)
    {
        timeout--;
        __delay_us(1);
    }
}
