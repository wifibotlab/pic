/* 
 * File:   I2C.h
 * Author: rbeal
 *
 * Created on May 9, 2017, 09:47 AM
 */

#ifndef I2C_H
#define	I2C_H

void init_I2C(void);

void I2C_read_multibytes(unsigned char device_addr, unsigned char addr, unsigned char* data, unsigned char nb_bytes);
char I2C_read_byte(unsigned char device_addr, unsigned char addr);
void I2C_write_byte(unsigned char device_addr, unsigned char addr, unsigned char data);

void I2C_busy();
void wait_ack();

#endif	/* I2C_H */

